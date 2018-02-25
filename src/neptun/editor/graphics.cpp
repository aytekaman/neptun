
#include "graphics.h"

//#include "gl3w/include/GL/gl3w.h"
#include "GLFW/glfw3.h"

#include <algorithm>
#include <set>

#include "neptun/main/color.h"
#include "neptun/main/material.h"
#include "neptun/main/mesh.h"
#include "neptun/main/scene.h"
#include "neptun/main/tet_mesh.h"
#include "neptun/main/texture.h"
#include "neptun/main/asset_importer.h"

#include <glm\gtx\transform.hpp>
#include <glm\gtx\euler_angles.hpp>
#include <glm\gtc\matrix_transform.hpp>

Graphics::Graphics()
{
    //Init();
}

void Graphics::Init()
{
    programID = LoadShaders("../../shaders/vertexShader.glsl", "../../shaders/fragmentShader.glsl");

    //Texture *texture = AssetImporter::ImportTexture("ball.jpg");

    // line drawing
    linesProgramID = LoadShaders("../../shaders/lineVertexShader.glsl", "../../shaders/lineFragmentShader.glsl");

    glGenVertexArrays(1, &linesVertexArrayID);
    glGenBuffers(2, linesVertexBufferIDs);

    glBindVertexArray(linesVertexArrayID);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferIDs[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferIDs[1]);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);

    // tri drawing
    trisProgramID = LoadShaders("../../shaders/triVertexShader.glsl", "../../shaders/triFragmentShader.glsl");
    glGenVertexArrays(1, &trisVertexArrayID);
    glGenBuffers(4, trisVertexBufferIDs);

    glBindVertexArray(trisVertexArrayID);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, 0);

    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[2]);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(3);
    glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[3]);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
}

void Graphics::Render(Scene *scene, bool show_tetrahedrons, RenderingMode rendering_mode)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);

    p = glm::perspective(glm::radians(45.0f), (float)(display_w - 360) / (display_h - 20), 0.1f, 1000.0f);

    double time = 0;

    glm::vec3 camTarget = scene->camTarget;
    glm::vec3 dir = glm::vec3(glm::cos(scene->camOrbitY), 0, glm::sin(scene->camOrbitY));

    dir = dir * glm::cos(scene->camOrbitX);

    dir.y = glm::sin(scene->camOrbitX);

    glm::vec3 camPos = camTarget +  dir * scene->camDist;

    v = glm::lookAt(camPos, camTarget, glm::vec3(0, 1, 0));

    glm::mat4 m = glm::mat4(1.0f);

    glm::mat4 mvp = p * v * m;

    // bbbox


    // line drawing
    glUseProgram(linesProgramID);

    GLuint MatrixID = glGetUniformLocation(linesProgramID, "MVP");

    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    glBindVertexArray(linesVertexArrayID);

    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferIDs[0]);
    glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(glm::vec3), (void*)&lineVertices[0].x, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferIDs[1]);
    glBufferData(GL_ARRAY_BUFFER, lineColors.size() * sizeof(glm::vec4), (void*)&lineColors[0].x, GL_DYNAMIC_DRAW);

    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-2.0, -2.0);
    glDrawArrays(GL_LINES, 0, lineVertices.size());
    glDisable(GL_POLYGON_OFFSET_LINE);

    if(show_tetrahedrons)
        UpdateTetMeshData(scene);
    // tri drawing
    glUseProgram(trisProgramID);

    MatrixID = glGetUniformLocation(trisProgramID, "MVP");

    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    glBindVertexArray(trisVertexArrayID);

    if (scene->tet_mesh && show_tetrahedrons)
    {
        int max = 0;

        while (scene->tet_mesh->slice > tet_datas[max].z)
            max++;

        max = glm::clamp(max, 0, (int)(scene->tet_mesh->m_tets.size() - 1));

        //auto max = std::lower_bound(tet_datas.begin(), tet_datas.end(), scene->tet_mesh->slice)

        glDrawArrays(GL_TRIANGLES, 0, max * 3 * 4);

    }



    glUseProgram(programID);
    //glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);

    for (int i = 0; i < scene->sceneObjects.size(); i++)
    {
        if (!scene->sceneObjects[i]->isVisible || scene->sceneObjects[i]->mesh == NULL)
            continue;

        Mesh *mesh = scene->sceneObjects[i]->mesh;

        if (mesh->isDirty)
        {
            std::unordered_map<Mesh*, RenderData>::const_iterator result = renderDatas.find(mesh);

            if (result == renderDatas.end())
            {
                CreateRenderData(mesh);
            }

            SendMeshData(mesh);

            mesh->isDirty = false;
        }

        std::unordered_map<Mesh*, RenderData>::const_iterator result = renderDatas.find(mesh);
        RenderData renderData = result->second;

        Texture *texture = scene->sceneObjects[i]->material->texture;

        if (texture)
        {
            if (texture->is_dirty)
            {
                std::unordered_map<Texture*, GLuint>::const_iterator result = texture_handles.find(texture);

                if (result == texture_handles.end())
                {
                    CreateTextureData(texture);
                }

                SendTextureData(texture);

                texture->is_dirty = false;
            }

            std::unordered_map<Texture*, GLuint>::const_iterator result2 = texture_handles.find(texture);
            GLint texture_id = result2->second;
            glBindTexture(GL_TEXTURE_2D, texture_id);
        }

        glm::mat4 t = glm::translate(glm::mat4(1.0f), scene->sceneObjects[i]->pos);
        glm::vec3 rot = glm::radians(scene->sceneObjects[i]->rot);
        glm::mat4 r = glm::eulerAngleYXZ(rot.y, rot.x, rot.z);
        glm::mat4 s = glm::scale(glm::mat4(1.0), scene->sceneObjects[i]->scale);

        s[3][3] = 1;

        mvp = p * v * t * r * s;

        GLuint MatrixID = glGetUniformLocation(programID, "MVP");
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

        MatrixID = glGetUniformLocation(programID, "N");

        glm::mat4 n = r;

        n = glm::transpose(glm::inverse(n));

        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &n[0][0]);

        GLuint textured = glGetUniformLocation(programID, "textured");
        glUniform1i(textured, mesh->uvs.size() > 0 && texture);

        GLuint rendering_mode_glsl = glGetUniformLocation(programID, "rendering_mode");
        glUniform1i(rendering_mode_glsl, rendering_mode == RenderingMode::SolidWireframe);

        GLuint wire_color_glsl = glGetUniformLocation(programID, "wire_color");
        glUniform3fv(wire_color_glsl, 1, &(scene->sceneObjects[i]->color.x));

        glBindVertexArray(renderData.vertexArrayID);
        

        glDrawArrays(GL_TRIANGLES, 0, mesh->vertexCount);
    }

    lineVertices.clear();
    lineColors.clear();

    //triVertices.clear();
    //triNormals.clear();
    //triColors.clear();
}

void Graphics::DrawLine(glm::vec3 start, glm::vec3 end, glm::vec4 color)
{
    lineVertices.push_back(start);
    lineVertices.push_back(end);

    lineColors.push_back(color);
    lineColors.push_back(color);
}

void Graphics::DrawTri(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec4 color)
{
    triVertices.push_back(a);
    triVertices.push_back(b);
    triVertices.push_back(c);

    glm::vec3 n = glm::normalize(glm::cross(a - b, b - c));

    triNormals.push_back(n);
    triNormals.push_back(n);
    triNormals.push_back(n);

    triColors.push_back(color);
    triColors.push_back(color);
    triColors.push_back(color);
}



void Graphics::DrawTet(glm::vec3 p[4], glm::vec4 color)
{
    //for (int i = 0; i < 3; i++)
    //	for (int j = i + 1; j < 4; j++)
    //		DrawLine(p[i], p[j], color);
    
    color = color * 0.9f;
    
    DrawTri(p[1], p[0], p[2], color);
    DrawTri(p[0], p[1], p[3], color);
    DrawTri(p[2], p[0], p[3], color);
    DrawTri(p[1], p[2], p[3], color);
}

void Graphics::UpdateTetMeshData(const Scene* scene)
{
    TetMesh* tet_mesh = scene->tet_mesh;

    if (!tet_mesh || !tet_mesh->is_dirty)
        return;

    triVertices.clear();
    triNormals.clear();
    triColors.clear();
    tet_datas.clear();

    for (int i = 0; i < tet_mesh->m_tets.size(); ++i)
    {
        unsigned int *idx = tet_mesh->m_tets[i].v;

        glm::vec3 p[4];

        for (int j = 0; j < 4; ++j)
        {
            p[j] = tet_mesh->m_points[idx[j]];
        }

        float z = (p[0].z + p[1].z + p[2].z + p[3].z) * 0.25f;

        TetData td;
        td.tet_idx = i;
        td.z = z;

        tet_datas.push_back(td);
    }

    std::sort(tet_datas.begin(), tet_datas.end(), [](TetData a, TetData b) {return a.z < b.z; });

    for (int i = 0; i < tet_datas.size(); ++i)
    {
        unsigned int *idx = tet_mesh->m_tets[tet_datas[i].tet_idx].v;

        glm::vec3 p[4];

        for (int j = 0; j < 4; ++j)
        {
            p[j] = tet_mesh->m_points[idx[j]];
        }

        DrawTet(p, Color::lerp(Color::turquoise, Color::yellow, Color::melon, (float)tet_datas[i].tet_idx / tet_mesh->m_tets.size()));
    }


    //if (c.z < 0)
        //DrawTet(p, Color::lerp(Color::turquoise, Color::yellow, Color::melon, (float)i / tet_mesh->num_tets));

    if (triVertices.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[0]);
        glBufferData(GL_ARRAY_BUFFER, triVertices.size() * sizeof(glm::vec3), (void*)&triVertices[0].x, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[1]);
        glBufferData(GL_ARRAY_BUFFER, triNormals.size() * sizeof(glm::vec3), (void*)&triNormals[0].x, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[2]);
        glBufferData(GL_ARRAY_BUFFER, triColors.size() * sizeof(glm::vec4), (void*)&triColors[0].x, GL_DYNAMIC_DRAW);

        std::vector<glm::vec3> bary(triVertices.size());
        for (size_t i = 0; i < triVertices.size(); ++i)
            bary[i][i % 3] = 1;

        glBindBuffer(GL_ARRAY_BUFFER, trisVertexBufferIDs[3]);
        glBufferData(GL_ARRAY_BUFFER, triVertices.size() * sizeof(glm::vec3), (void*)&bary[0].x, GL_STATIC_DRAW);
    }

    tet_mesh->is_dirty = false;
}

void Graphics::DrawCube(glm::vec3 min, glm::vec3 max, glm::vec4 color)
{
glm::vec3 a(min.x, min.y, min.z);
    glm::vec3 b(min.x, min.y, max.z);
    glm::vec3 c(max.x, min.y, max.z);
    glm::vec3 d(max.x, min.y, min.z);

    glm::vec3 e(min.x, max.y, min.z);
    glm::vec3 f(min.x, max.y, max.z);
    glm::vec3 g(max.x, max.y, max.z);
    glm::vec3 h(max.x, max.y, min.z);

    DrawLine(a, b, color);
    DrawLine(b, c, color);
    DrawLine(c, d, color);
    DrawLine(d, a, color);

    DrawLine(e, f, color);
    DrawLine(f, g, color);
    DrawLine(g, h, color);
    DrawLine(h, e, color);

    DrawLine(a, e, color);
    DrawLine(b, f, color);
    DrawLine(c, g, color);
    DrawLine(d, h, color);
}

void Graphics::CreateRenderData(Mesh * mesh)
{
    RenderData renderData;

    glGenVertexArrays(1, &(renderData.vertexArrayID));
    glGenBuffers(4, renderData.vertexBufferIDs);

    glBindVertexArray(renderData.vertexArrayID);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[0]);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[1]);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, 0);

    if (mesh->uvs.size() > 0)
    {
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[2]);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);
    }

    glEnableVertexAttribArray(3);
    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[3]);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);

    std::pair<Mesh*, RenderData> pair(mesh, renderData);

    renderDatas.insert(pair);
}

void Graphics::SendMeshData(Mesh *mesh)
{
    std::unordered_map<Mesh*, RenderData>::const_iterator result = renderDatas.find(mesh);

    RenderData renderData = result->second;

    glBindVertexArray(renderData.vertexArrayID);

    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[0]);
    glBufferData(GL_ARRAY_BUFFER, mesh->vertexCount * sizeof(glm::vec3), (void*)&mesh->vertices[0].x, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[1]);
    glBufferData(GL_ARRAY_BUFFER, mesh->vertexCount * sizeof(glm::vec3), (void*)&mesh->normals[0].x, GL_STATIC_DRAW);

    if (mesh->uvs.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[2]);
        glBufferData(GL_ARRAY_BUFFER, mesh->vertexCount * sizeof(glm::vec2), (void*)&mesh->uvs[0].x, GL_STATIC_DRAW);
    }

    std::vector<glm::vec3> bary(mesh->vertexCount);
    for (size_t i = 0; i < mesh->vertexCount; ++i)
        bary[i][i % 3] = 1;

    glBindBuffer(GL_ARRAY_BUFFER, renderData.vertexBufferIDs[3]);
    glBufferData(GL_ARRAY_BUFFER, mesh->vertexCount * sizeof(glm::vec3), (void*)&bary[0].x, GL_STATIC_DRAW);
}

void Graphics::CreateTextureData(Texture * texture)
{
    GLuint texture_id;

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);

    glGenTextures(1, &texture_id);

    glBindTexture(GL_TEXTURE_2D, texture_id);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture->w, texture->h, 0, GL_RGB, GL_UNSIGNED_BYTE, texture->pixels);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    std::pair<Texture*, GLuint> pair(texture, texture_id);

    texture_handles.insert(pair);
}

void Graphics::SendTextureData(Texture * texture)
{
    std::unordered_map<Texture*, GLuint>::const_iterator result = texture_handles.find(texture);

    GLuint texture_id= result->second;

    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture->w, texture->h, 0, GL_RGB, GL_UNSIGNED_BYTE, texture->pixels);
}

GLuint Graphics::LoadShaders(std::string vertexFilePath, std::string fragmentFilePath)
{
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    // Read the Vertex Shader code from the file
    std::string VertexShaderCode;
    std::ifstream VertexShaderStream(vertexFilePath, std::ios::in);
    if (VertexShaderStream.is_open()) {
        std::string Line = "";
        while (getline(VertexShaderStream, Line))
            VertexShaderCode += "\n" + Line;
        VertexShaderStream.close();
    }
    else {
        printf("Impossible to open %s. Are you in the right directory ? Don't forget to read the FAQ !\n", vertexFilePath);
        getchar();
        return 0;
    }

    // Read the Fragment Shader code from the file
    std::string FragmentShaderCode;
    std::ifstream FragmentShaderStream(fragmentFilePath, std::ios::in);
    if (FragmentShaderStream.is_open()) {
        std::string Line = "";
        while (getline(FragmentShaderStream, Line))
            FragmentShaderCode += "\n" + Line;
        FragmentShaderStream.close();
    }

    GLint Result = GL_FALSE;
    int InfoLogLength;

    // Compile Vertex Shader
    printf("Compiling shader : %s\n", vertexFilePath.c_str());
    char const * VertexSourcePointer = VertexShaderCode.c_str();
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
    glCompileShader(VertexShaderID);

    // Check Vertex Shader
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
    }

    // Compile Fragment Shader
    printf("Compiling shader : %s\n", fragmentFilePath.c_str());
    char const * FragmentSourcePointer = FragmentShaderCode.c_str();
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
    glCompileShader(FragmentShaderID);

    // Check Fragment Shader
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
        printf("%s\n", &FragmentShaderErrorMessage[0]);
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
        std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
        glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);
    }


    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
}