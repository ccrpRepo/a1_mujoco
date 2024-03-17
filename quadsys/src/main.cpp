#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>

#include "interface/IOmujoco.h"
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "wbc_controller/wbc_controller.h"

#define COMPILE_WITH_SIMULATION
#define ROBOT_TYPE_A1

std::string _robot_name;

mjModel *m;
mjData *d;      // MuJoCo data
mjvCamera cam;  // abstract camera
mjvOption opt;  // visualization options     此结构包含启用和禁用各种元素可视化的选项。
mjvScene scn;   // abstract scene            该结构包含在 OpenGL 中渲染 3D 场景所需的一切。
mjrContext con; // custom GPU context        该结构包含自定义 OpenGL 渲染上下文，以及上传到 GPU 的所有 OpenGL 资源的 ID。
GLFWwindow *window;

// mouse interaction
bool button_left;
bool button_middle;
bool button_right;
double lastx;
double lasty;

bool running = true;

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;
    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

void Init_window()
{
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "CRP", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);              // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150); // model-specific context
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    double arr_view[] = {90, -90, 10, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

int main(int argc, char **argv)
{
    long long int timer = 0;
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    IOinterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    std::cout << "The control interface for mujoco simulation" << std::endl;
    _robot_name = "A1_dog";
    std::cout << "robot_name: " << _robot_name << std::endl;

    char filename[] = "/home/crp/git/quad_sys2/robot/a1_description/a1mjcf.xml";
    char error[1000] = "Could not load binary model";
    lastx = 0;
    lasty = 0;
    button_left = false;
    button_middle = false;
    button_right = false;

    m = mj_loadXML(filename, 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);
    d = mj_makeData(m);
    Init_window();

    ioInter = new IOmujoco(d);
    ctrlPlat = CtrlPlatform::MUJOCO;

#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT

    a1Robot *a1 = new a1Robot();
    Dynamics *dy = new Dynamics(a1);
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter, dy);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;
    #ifdef ROBOT_TYPE_A1
        ctrlComp->robotModel = new A1Robot();
    #endif
    // #ifdef ROBOT_TYPE_Go1
    //     ctrlComp->robotModel = new Go1Robot();
    // #endif

    ctrlComp->waveGen = new WaveGenerator(0.5, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot 0.45
                                                                            // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    //  ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);
    
    while (running)
    {
        running = !glfwWindowShouldClose(window);
        mjtNum simstart = d->time;
        while ((d->time - simstart) < 1.0 / 60.0)
        {
            if(d->time>1.0)
                ctrlFrame.run();
            // std::cout << std::endl;
            mj_step(m, d);
        }
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render  更新场景并渲染
        opt.frame = mjFRAME_WORLD; // 显示轴线
        // cam.lookat[0] = d->qpos[3]; // 视角跟随物体的x轴
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n", cam.azimuth, cam.elevation, cam.distance, cam.lookat[0], cam.lookat[1], cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteModel(m);
    mj_deleteData(d);
    delete ctrlComp;
    return 0;
}