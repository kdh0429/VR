#pragma once

// #include "hmdHandler_sphere.h"
#include "hmdHandler.h"
#include <thread>


int main(int argc, char* argv[])
{   
   
    HMD* hmdSystem = new HMD(argc, argv);
    
    hmdSystem->init();
       
    // std::thread threads[2];
    // threads[0] = std::thread(&HMD::RunMainLoop,*hmdSystem);
    // threads[1] = std::thread(&HMD::RunRosLoop,*hmdSystem);

    // threads[0].join();
    // threads[1].join();

    hmdSystem->RunMainLoop();
    


    //glfwInit();
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    //

    //GLFWwindow* window = glfwCreateWindow(800, 600, "Learn OpenGL", NULL, NULL);
    //if (window == NULL) {
    //    cout << "Window Creation failed" << endl;
    //    glfwTerminate();
    //    return -1;

    //}

    //glfwMakeContextCurrent(window);
    //if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    //    cout << "Failed to initialized GLAD" << endl;
    //    //return -1;
    //}

    //glViewport(0, 0, 800, 600);

    //unsigned int vertexShader, fragmentShader;

    //vertexShader = glCreateShader(GL_VERTEX_SHADER);
    //fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

    //glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    //glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    //glCompileShader(vertexShader);
    //glCompileShader(fragmentShader);

    //int vertSuccess, fragSuccess;
    //char vertInfoLog[512], fragInfoLog[512];
    //glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &vertSuccess);
    //glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &fragSuccess);

    //if (!vertSuccess)
    //{
    //    glGetShaderInfoLog(vertexShader, 512, NULL, vertInfoLog);
    //    cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << vertInfoLog << endl;
    //}
    //if (!fragSuccess)
    //{
    //    glGetShaderInfoLog(fragmentShader, 512, NULL, fragInfoLog);
    //    cout << "ERROR::SHADER::FRAG::COMPILATION_FAILED\n" << fragInfoLog<< endl;
    //}


    //unsigned int shaderProgram;
    //int programSuccess;
    //char programInfoLog[512];
    //shaderProgram = glCreateProgram();
    //glAttachShader(shaderProgram, vertexShader);
    //glAttachShader(shaderProgram, fragmentShader);
    //glLinkProgram(shaderProgram);
    //glGetProgramiv(shaderProgram, GL_LINK_STATUS, &programSuccess);

    //if (!programSuccess) {
    //    glGetProgramInfoLog(shaderProgram, 512, NULL, programInfoLog);
    //    cout << "ERROR::SHADER::PROGRAM::LINKING ERROR" << programInfoLog << endl;
    //}

    //glUseProgram(shaderProgram);
    //glDeleteShader(vertexShader);
    //glDeleteShader(fragmentShader);


    //unsigned int VBO, VAO;
    //glGenBuffers(1, &VBO);
    //glGenVertexArrays(1, &VAO);
    //glBindVertexArray(VAO);

    //glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    //glEnableVertexAttribArray(0);

    //float timeValue;
    //float greenValue;
    //int vertexColorLocation;

    //while (!glfwWindowShouldClose(window)) {

    //    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    //    glClear(GL_COLOR_BUFFER_BIT);

    //    timeValue = glfwGetTime();
    //    greenValue = (sin(timeValue) / 2.0f) + 0.5f;
    //    vertexColorLocation = glGetUniformLocation(shaderProgram, "ourColor");
    //    glUniform4f(vertexColorLocation, 0.0f, greenValue, 0.0f, 1.0f);

    // 
    //    glDrawArrays(GL_TRIANGLES, 0, 3);

    //    glfwSwapBuffers(window);
    //    glfwPollEvents();
    //}
    return 0;
}


