//
// Created by qixiaofeng on 2021/4/7.
//
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
// sudo apt install -y libglew-dev
#include <GL/glew.h>
#include <SDL2/SDL.h>

#define WinWidth 1000
#define WinHeight 1000

static uint32_t const g_base_flag = SDL_WINDOW_OPENGL;

void toggle_full_screen(SDL_Window *window) {
    static uint32_t const full_screen_flag = SDL_WINDOW_FULLSCREEN_DESKTOP;
    static uint32_t const flags_with_full_screen = g_base_flag | full_screen_flag;

    static bool is_full_screen = false;

    if (is_full_screen) {
        SDL_SetWindowFullscreen(window, g_base_flag);
        is_full_screen = false;
    } else {
        SDL_SetWindowFullscreen(window, flags_with_full_screen);
        is_full_screen = true;
    }
}

void read_file(char const *file_path, char *buffer, int buffer_size) {
    // https://stackoverflow.com/questions/238603/how-can-i-get-a-files-size-in-c
    /**
    fseek(fp, 0L, SEEK_END);
    sz = ftell(fp); // Tell size of the file
    // Seek to the file beginning
    fseek(fp, 0L, SEEK_SET);
    // or rewind(fp);
    */
    FILE *file = fopen(file_path, "r");
    fread(buffer, sizeof(char), buffer_size, file);
    fclose(file);
}

GLuint LoadShaders(char const *vertex_file_path, char const *fragment_file_path) {
    // Create the shaders
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    // Read the Vertex Shader code from the file
    char VertexShaderCode[1024];
    read_file(vertex_file_path, VertexShaderCode, 1024);
    // Read the Fragment Shader code from the file
    char FragmentShaderCode[1024];
    read_file(fragment_file_path, FragmentShaderCode, 1024);

    GLint Result = GL_FALSE;
    int InfoLogLength;

    // Compile Vertex Shader
    printf("Compiling shader : %s\n%s\n", vertex_file_path, VertexShaderCode);
    char const *VertexSourcePointer = VertexShaderCode;
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
    glCompileShader(VertexShaderID);

    // Check Vertex Shader
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        char *VertexShaderErrorMessage = malloc(sizeof(char) * (InfoLogLength + 1));
        glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, VertexShaderErrorMessage);
        printf("%s\n", &VertexShaderErrorMessage[0]);
        free(VertexShaderErrorMessage);
    }

    // Compile Fragment Shader
    printf("Compiling shader : %s\n", fragment_file_path);
    char const *FragmentSourcePointer = FragmentShaderCode;
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
    glCompileShader(FragmentShaderID);

    // Check Fragment Shader
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        char *FragmentShaderErrorMessage = malloc(sizeof(char) * (InfoLogLength + 1));
        glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, FragmentShaderErrorMessage);
        printf("%s\n", &FragmentShaderErrorMessage[0]);
        free(FragmentShaderErrorMessage);
    }

    // Link the program
    printf("Linking program\n");
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    // Check the program
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
    glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        char *ProgramErrorMessage = malloc(sizeof(char) * (InfoLogLength + 1));
        glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);
        free(ProgramErrorMessage);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
}

int main() {
    // SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("OpenGL Test", 0, 0, WinWidth, WinHeight, g_base_flag);
    assert(window);
    SDL_GLContext context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, context);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    char const *gl_version = (char *) glGetString(GL_VERSION);
    printf("OpenGL Version: %s\n", gl_version);

    glewInit();
    GLuint program_id = LoadShaders(
            "/home/qixiaofeng/Documents/git-repos/uinp/clanguages/linux-sdl2-canvas/shaders/first.vs",
            "/home/qixiaofeng/Documents/git-repos/uinp/clanguages/linux-sdl2-canvas/shaders/first.fs"
    );

    GLuint vertex_array_id;
    glGenVertexArrays(1, &vertex_array_id);
    glBindVertexArray(vertex_array_id);
    GLfloat const vertex_buffer_data[] = {
            -1.0f, -1.0f, 0.0f,
            1.0f, -1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
    };
    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_buffer_data), vertex_buffer_data, GL_STATIC_DRAW);

    bool Running = true;
    while (Running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        Running = false;
                        break;
                    case 'f':
                        toggle_full_screen(window);
                        break;
                    default:
                        break;
                }
            } else if (event.type == SDL_QUIT) {
                Running = false;
            }
        }

        glViewport(0, 0, WinWidth, WinHeight);
        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(program_id);

        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glDisableVertexAttribArray(0);

        SDL_GL_SwapWindow(window);
    }
    return 0;
}
