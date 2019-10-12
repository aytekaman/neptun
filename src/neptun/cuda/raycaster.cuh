#include "cuda.h"
#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include "neptun/main/basis.h"
#include "neptun/main/ray_tracer.h"
#include "neptun/main/scene.h"
#include "neptun/main/stats.h"
#include "neptun/main/tet_mesh.h"

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"

#include <iostream>
#include <stdio.h>
#include <string.h>


//TCDT utils
#define TCDT_CUDA_TCDT_USE_TEXTURES_OBJECTS

#define TCDT_LOOK_UP_CONSTANT

#define GET_NEXT_TETRA( _t )( ( _t & 0x3fffffff ) )
#define GET_NEXT_FACE( _t )( ( _t >> 30 ) & 0x3 )
#define GET_FACE_VERTEX( _f, _id )( ( c_orderVertices[_f] >> (2*_id) ) & 0x3 ) // Get face vertices ID
#define GET_CMP_VERTEX( _f )( _f ) // Get complementary vertex ID
#define GET_EXIT_FACE( _f, _id )( ( c_exitFace[_f] >> (2*_id) ) & 0x3 ) // Get exit face from id

#ifdef TCDT_CUDA_TCDT_USE_TEXTURES_OBJECTS
#define LOAD_TETRA( id )	tex1Dfetch<int2>(	tcdt->m_texTetras, id )
#define LOAD_VERTEX( id )	tex1Dfetch<float4>( tcdt->m_texVertices, id ) 
#else 
#define LOAD_TETRA( id )	tcdt->m_devTetras[id]
#define LOAD_VERTEX( id )	tcdt->m_devVertices[id]
#endif

#ifdef TCDT_LOOK_UP_CONSTANT
												//   3 2 1 0 /id 0 1 2
__constant__ char c_orderVertices[4] = {	//  00100111,    3 1 2 
											0x27,
											//  00110010,    2 0 3
											0x32,
											//  00010011,    3 0 1
											0x13,
											//  00100001     1 0 2
											0x21
};

//   3 2 1 0 /id 0 1 2
__constant__ char c_exitFace[4] = {	//  00111001,    1 2 3
										0x39,
										//  00101100,    0 3 2
										0x2c,
										//  00110100,    0 1 3
										0x34,
										//  00011000     0 2 1
										0x18
};
#endif

__device__
inline void swapvec2(glm::vec2 &a, glm::vec2 &b)
{
    glm::vec2 t = a;
    a = b;
    b = t;
}

__device__
inline void swap(unsigned int &a, unsigned int &b)
{
    unsigned int t = a;
    a = b;
    b = t;
}

__device__
inline void swap(int &a, int &b)
{
    int t = a;
    a = b;
    b = t;
}

__device__
inline float scalar_triple(glm::vec3 p, glm::vec3 q, glm::vec3 r)
{
    return glm::dot(p, glm::cross(q, r));
}

__device__
inline float crossv(const glm::vec2& a, const glm::vec2& b) { return a.x * b.y - a.y * b.x; }

__device__
inline float crossv(const glm::vec3& a, const glm::vec3& b) { return a.x * b.y - a.y * b.x; }

__host__
inline void print_cuda_error(char* operation_desc) { 
	cudaError_t error = cudaGetLastError(); 
	printf("%s: %s\n", operation_desc, error == cudaError::cudaSuccess ? "success" : cudaGetErrorString(error)); 
}

inline cudaError_t check_cuda(cudaError_t result)
{
#if defined(DEBUG) || defined(_DEBUG)
    if (result != cudaSuccess) {
        fprintf(stderr, "CUDA Runtime Error: %s\n", cudaGetErrorString(result));
        assert(result == cudaSuccess);
    }
#endif
    return result;
}