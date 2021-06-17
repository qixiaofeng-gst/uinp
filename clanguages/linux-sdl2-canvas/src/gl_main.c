//
// Created by qixiaofeng on 2021/4/7.
//
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
// sudo apt install -y libglew-dev
#include <GL/glew.h>
#include <SDL2/SDL.h>
#include "cglm/types.h"
#include "cglm/mat4.h"

#define WinWidth 1000
#define WinHeight 1000
#define print_vec4(v) printf("[%f, %f, %f, %f]\n", v[0], v[1], v[2], v[3])

/** https://www.codeproject.com/Articles/199525/Drawing-nearly-perfect-2D-line-segments-in-OpenGL */
/** https://www.shadertoy.com/browse */
/**
 * TODO Implement a shader platform like shadertoy.
 *
https://www.shadertoy.com/view/tsXBzS  -  tsXBzS(66, shining purple fragments)
    7ljGzR(7, towers) XsXXDn(21, shining) wlVGWd(44, purple water) MdX3zr(64, flame) Nt23zh(65, stars)
    ttKGDt(73, snow flakes) ldfyzl(79, rain on the water) llt3R4(86, red cycle) WtScDt(87, cloth pattern)
    tlGfzd(96, liberation) sllGRB(25 + 104, patterns) 4s2SRt(136, radar) 3l23Rh(145, protean clouds) NlsGDl(163, sea)
    WdVXWy(61 + 25 + 84 + 15, molten bismuth) 7tX3Dj(22 + 62 + 111, aquarelle) XslGRr(207, clouds)
    ltffzl(213, rain on the window) stlGWs(118 + 127, neon hall) 4dfGzs(299, voxel edges)
    slj3RR(241 + 111, morphing sphere) NlfGDX(421 + 95, bezier curve) 7lB3zz(357 + 489 * 3, blocks impact)
*/
static uint32_t const g_base_flag = SDL_WINDOW_OPENGL;
char gl_log_buffer[1024];

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
    uint64_t readed_count = fread(buffer, sizeof(char), buffer_size, file);
    if (feof(file)) {
        buffer[readed_count] = '\0'; /** XXX Something strange happens while missing this. */
    } else {
        printf("The file is not fully readed. There might be runtime issues.\n");
    }
    fclose(file);
}

void load_shader(GLuint shader_id, char const *source_path) {
    char shader_source[1024];
    char const *source_pointer = shader_source;
    int gl_log_length = 0;
    GLint gl_operation_result = GL_FALSE;

    // Compile Shader
    read_file(source_path, shader_source, 1024);
    glShaderSource(shader_id, 1, &source_pointer, NULL);
    glCompileShader(shader_id);
    // Check Shader
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &gl_operation_result);
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &gl_log_length);
    if (gl_log_length > 0) {
        glGetShaderInfoLog(shader_id, gl_log_length, NULL, gl_log_buffer);
        printf(
                "Failed to compile shader : %s\n%s\nGL log:%s\n",
                source_path, shader_source, gl_log_buffer
        );
    }
}

GLuint load_shaders(char const *vertex_file_path, char const *fragment_file_path) {
    GLint gl_operation_result = GL_FALSE;
    int gl_log_length;

    // Create the shaders
    GLuint vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    load_shader(vertex_shader_id, vertex_file_path);
    load_shader(fragment_shader_id, fragment_file_path);

    // Link the program
    GLuint program_id = glCreateProgram();
    glAttachShader(program_id, vertex_shader_id);
    glAttachShader(program_id, fragment_shader_id);
    glLinkProgram(program_id);
    // Check the program
    glGetProgramiv(program_id, GL_LINK_STATUS, &gl_operation_result);
    glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &gl_log_length);
    if (gl_log_length > 0) {
        glGetProgramInfoLog(program_id, gl_log_length, NULL, gl_log_buffer);
        printf("Failed to link program. GL log:\n%s\n", gl_log_buffer);
    }

    glDetachShader(program_id, vertex_shader_id);
    glDetachShader(program_id, fragment_shader_id);
    glDeleteShader(vertex_shader_id);
    glDeleteShader(fragment_shader_id);
    return program_id;
}

int main() {
    /** Belows initialize SDL and GL. */
    // SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("OpenGL Test", 0, 0, WinWidth, WinHeight, g_base_flag);
    assert(window);
    SDL_GLContext context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, context);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    char const *gl_version = (char *) glGetString(GL_VERSION);
    printf("OpenGL Version: %s\n", gl_version);

    glewInit();

    #define m__folder "/home/qixiaofeng/Documents/git-repos/uinp/clanguages/linux-sdl2-canvas/shaders/"
    GLuint program_id = load_shaders(m__folder"first.vs", m__folder"first.fs");
    #undef m__folder

    /** Belows create a square mesh. */
    GLuint vertex_array_id;
    glGenVertexArrays(1, &vertex_array_id);
    glBindVertexArray(vertex_array_id);
    GLfloat const vertex_buffer_data[] = {
            0.99f, 0.99f, 0.0f,
            0.99f, -.99f, 0.0f,
            -.99f, 0.99f, 0.0f,
            -.99f, -.99f, 0.0f,
    };
    GLuint vertex_buffer;
    glGenBuffers(1, &vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_buffer_data), vertex_buffer_data, GL_STATIC_DRAW);

    /** Belows create a texture. */
    uint8_t texture_data[16 * 16 * 3];
    memset(texture_data, 0xff, sizeof(texture_data));
    GLuint texture_id;
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 16, 16, 0, GL_BGR, GL_UNSIGNED_BYTE, texture_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    /** Belows create UV for the square mesh above. */
    GLfloat const uv_buffer_data[] = {
            1.0f, 0.0f,
            1.0f, 1.0f,
            0.0f, 0.0f,
            0.0f, 1.0f,
    };
    GLuint uv_buffer;
    glGenBuffers(1, &uv_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, uv_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(uv_buffer_data), uv_buffer_data, GL_STATIC_DRAW);

    /** Belows play with GLM.  */
    mat4 translation_matrix = {
            {1, 0, 0, 10},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1},
    };
    glm_mat4_transpose(translation_matrix);

    vec4 vector = {10, 10, 10, 1};
    vec4 transformed_vector;

    glm_mat4_mulv(translation_matrix, vector, transformed_vector);
    print_vec4(transformed_vector);

    mat4 scaling_matrix = {
            {2, 0, 0, 0},
            {0, 2, 0, 0},
            {0, 0, 2, 0},
            {0, 0, 0, 1},
    };
    glm_mat4_transpose(scaling_matrix);
    glm_mat4_mulv(scaling_matrix, vector, transformed_vector);
    print_vec4(transformed_vector);

    /** Belows are main loop. */
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
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uv_buffer);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);

        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        SDL_GL_SwapWindow(window);
    }

    glDeleteBuffers(1, &vertex_buffer);
    glDeleteBuffers(1, &uv_buffer);
    glDeleteProgram(program_id);
    glDeleteTextures(1, &texture_id);
    glDeleteVertexArrays(1, &vertex_array_id);

    SDL_DestroyWindow(window);
    return 0;
}
