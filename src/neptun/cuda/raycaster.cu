#include "raycaster.cuh";
#include <chrono>
#include <ctime>

//========= Device data =========
Ray* d_rays;
IntersectionData* d_intersectdata;
glm::vec3* d_points;
TetMesh32::Tet32* d_tet32s;
TetMesh20::Tet20* d_tet20s;
TetMesh16::Tet16* d_tet16s;
TetMeshSctp::TetSctp* d_tetSctps;

//--------- Tet96 data------------
// 4 int2 per tetra (1 per face):
// 1 int 2 = m_semantics, m_nextTetraFace
int2* d_tet96s;
float4* d_vertices;// 1 float4 per vertex
cudaTextureObject_t t_tet96s;
cudaTextureObject_t t_vertices;
int num_int2_d_tets;
int num_float4_d_vertices;

ConstrainedFace* d_cons_faces;
Face* d_faces;
glm::ivec2* d_res;
SourceTet* d_source_tet;
Scene* d_scene;

//========= Init data =========
glm::vec3 old_target(NAN, NAN, NAN);//To force init of d_source_tet and d_scene
float old_dist = 0.0f;
float old_orbit_x = 0.0f;
float old_orbit_y = 0.0f;
unsigned int old_size = 0;

//======== Asynch computation variables =========
cudaStream_t* streams;
const int NSTREAMS = 3;


//=============== TCDT utils =====================
//#define TCDT_CUDA_TCDT_USE_TEXTURES_OBJECTS

#define TCDT_LOOK_UP_CONSTANT
#define TCDT_MAGIC_ERROR -1


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

#define GET_NEXT_TETRA( _t )( ( _t & 0x3fffffff ) )
#define GET_NEXT_FACE( _t )( ( _t >> 30 ) & 0x3 )
#define GET_FACE_VERTEX( _f, _id )( ( c_orderVertices[_f] >> (2*_id) ) & 0x3 ) // Get face vertices ID
#define GET_CMP_VERTEX( _f )( _f ) // Get complementary vertex ID
#define GET_EXIT_FACE( _f, _id )( ( c_exitFace[_f] >> (2*_id) ) & 0x3 ) // Get exit face from id

#ifdef TCDT_CUDA_TCDT_USE_TEXTURES_OBJECTS
#define LOAD_TETRA( id )	tex1Dfetch<int2>(	t_tet96s, id )
#define LOAD_VERTEX( id )	tex1Dfetch<float4>( t_vertices, id ) 
#else 
#define LOAD_TETRA( id )	d_tet96s[id]
#define LOAD_VERTEX( id )	d_vertices[id]
#endif

__device__
int get_exit_face(const TetMesh80::Plucker& plRay,
	const int idTetra, const int idEntryFace, float4* d_vertices/*cudaTextureObject_t t_vertices*/) {
	int idExit;

	const float4 cmpV = LOAD_VERTEX(idTetra + GET_CMP_VERTEX(idEntryFace));

	// Translate ray to vOrig
	const float4 rMom = make_float4(
		(plRay.m_Pi[1] * cmpV.z - plRay.m_Pi[2] * cmpV.y) + plRay.m_Pi[3],
		(plRay.m_Pi[2] * cmpV.x - plRay.m_Pi[0] * cmpV.z) + plRay.m_Pi[4],
		(plRay.m_Pi[0] * cmpV.y - plRay.m_Pi[1] * cmpV.x) + plRay.m_Pi[5],
		0.f);

	// Perform first Plucker test (with 0)
	// and get next Plucker line to test
	float4 edge = LOAD_VERTEX(idTetra + GET_FACE_VERTEX(idEntryFace, 2));
	edge.x -= cmpV.x;
	edge.y -= cmpV.y;
	edge.z -= cmpV.z;

	// Get next edge [1..2] according to side's result
	idExit = ((rMom.x * edge.x
		+ rMom.y * edge.y
		+ rMom.z * edge.z) < 0.0f);

	// Perform second and last test and get exit face [0..3]
	edge = LOAD_VERTEX(idTetra + GET_FACE_VERTEX(idEntryFace, idExit));
	edge.x -= cmpV.x;
	edge.y -= cmpV.y;
	edge.z -= cmpV.z;

	// idNextEdge is used as an exit ID
	idExit += ((rMom.x * edge.x
		+ rMom.y * edge.y
		+ rMom.z * edge.z) >= 0.0f);

	// Real idExit
	idExit = GET_EXIT_FACE(idEntryFace, idExit);

	return idExit;
}

//================= Inits a ray for corresponding pixel & return index for the output of that vertex data ==================
__device__
int init_ray(Scene& scene, glm::ivec2& resolution, int tile_size, int idx, glm::vec3& ray_origin, glm::vec3& ray_dir)
{
	const glm::vec3 camTarget = scene.camTarget;
	glm::vec3 dir = glm::vec3(glm::cos(scene.camOrbitY), 0, glm::sin(scene.camOrbitY));

	dir = dir * glm::cos(scene.camOrbitX);
	dir.y = glm::sin(scene.camOrbitX);

	glm::vec3 cam_pos = camTarget + dir * scene.camDist;

	const glm::vec3 cam_forward = glm::normalize(scene.camTarget - cam_pos);
	const glm::vec3 cam_right = -glm::normalize(glm::cross(glm::vec3(0, 1, 0), cam_forward));
	const glm::vec3 cam_down = glm::cross(cam_forward, cam_right);

	const glm::vec3 cam_up = glm::cross(cam_forward, cam_right);

	const float aspect = (float)resolution.x / resolution.y;
	const float scale_y = glm::tan(glm::pi<float>() / 8);

	const glm::vec3 bottom_left = cam_pos + cam_forward - cam_up * scale_y - cam_right * scale_y * aspect;
	const glm::vec3 up_step = (cam_up * scale_y * 2.0f) / (float)resolution.y;

	const glm::vec3 top_left = cam_pos + cam_forward - cam_down * scale_y - cam_right * scale_y * aspect;
	const glm::vec3 right_step = (cam_right * scale_y * 2.0f * aspect) / (float)resolution.x;
	const glm::vec3 down_step = (cam_down * scale_y * 2.0f) / (float)resolution.y;

	const int tile_count_x = (resolution.x + tile_size - 1) / tile_size;
	const int tile_count_y = (resolution.y + tile_size - 1) / tile_size;
	const int max_job_index = tile_count_x * tile_count_y;

	glm::ivec2 rect_min = glm::ivec2((idx / (tile_size * tile_size) % tile_count_x) * tile_size, (idx / (tile_size * tile_size) / tile_count_x) * tile_size);
	glm::ivec2 rect_max = rect_min + glm::ivec2(tile_size, tile_size);
	rect_max = (glm::min)(rect_max, resolution);

	int tile_offset = idx - rect_min.x * tile_size - rect_min.y * (resolution.x);

	glm::ivec2 pixel_coords = glm::ivec2(rect_min.x + tile_offset % tile_size, rect_min.y + tile_offset / tile_size);

	int outputindex = pixel_coords.x + pixel_coords.y * resolution.x;

	//j = rect_min.y
	//i = rect_min.x
	ray_origin = cam_pos;
	ray_dir = glm::normalize(top_left + right_step * (float)pixel_coords.x + down_step * (float)pixel_coords.y - ray_origin);

	return outputindex;
}

//============================ Ray Caster Kernels where rays are initialized ===============================

//----------------------------------------- For TetMeshScTP -------------------------------------------------
__global__
void ray_cast_kernel(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int offset, int tile_size,
	glm::vec3* points, TetMeshSctp::TetSctp* tets, ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < resolution.x * resolution.y)
	{
		glm::vec3 ray_origin, ray_dir;
		int outputindex = init_ray(scene, resolution, tile_size, idx, ray_origin, ray_dir);

		unsigned int id[4];
		glm::vec3 p[4];

		for (int i = 0; i < 4; i++)
		{
			id[i] = source_tet.v[i];
			p[i] = points[id[i]] - ray_origin;
		}

		//p[0] = A, p[1] = B, p[2] = C, p[3] = D
		float QAB = scalar_triple(ray_dir, p[0], p[1]); // A B
		float QBC = scalar_triple(ray_dir, p[1], p[2]); // B C
		float QAC = scalar_triple(ray_dir, p[0], p[2]); // A C
		float QAD = scalar_triple(ray_dir, p[0], p[3]); // A D
		float QBD = scalar_triple(ray_dir, p[1], p[3]); // B D
		float QCD = scalar_triple(ray_dir, p[2], p[3]); // C D

		float sQAB = copysignf(1.0f, QAB); // A B
		float sQBC = copysignf(1.0f, QBC); // B C
		float sQAC = copysignf(1.0f, QAC); // A C
		float sQAD = copysignf(1.0f, QAD); // A D
		float sQBD = copysignf(1.0f, QBD); // B D
		float sQCD = copysignf(1.0f, QCD); // C D

		int outIdx = -1;
		// ABC
		if ((sQAB != 0 && sQAC != 0 && sQBC != 0) &&
			(sQAB < 0 && sQAC > 0 && sQBC < 0))
		{
			outIdx = 3;
		}
		// BAD
		if ((sQAB != 0 && sQAD != 0 && sQBD != 0) &&
			(sQAB > 0 && sQAD < 0 && sQBD > 0))
		{
			outIdx = 2;
		}
		// CDA
		if ((sQAD != 0 && sQAC != 0 && sQCD != 0) &&
			(sQAD > 0 && sQAC < 0 && sQCD < 0))
		{
			outIdx = 1;
		}
		// DCB
		if ((sQBC != 0 && sQBD != 0 && sQCD != 0) &&
			(sQBC > 0 && sQBD < 0 && sQCD > 0))
		{
			outIdx = 0;
		}
		if (outIdx == -1)
		{
			output[outputindex].hit = false;
			return;
		}

		int index = source_tet.n[outIdx];
		bool hit_cons_face = false;
		int counter = 0;

		while (!hit_cons_face && index > -1 && counter < 1000)
		{
			for (int i = 0; i < 4; i++)
			{
				id[i] = tets[index].v[i];
				p[i] = points[id[i]] - ray_origin;
			}

			QAB = scalar_triple(ray_dir, p[0], p[1]); // A B
			QBC = scalar_triple(ray_dir, p[1], p[2]); // B C
			QAC = scalar_triple(ray_dir, p[0], p[2]); // A C
			QAD = scalar_triple(ray_dir, p[0], p[3]); // A D
			QBD = scalar_triple(ray_dir, p[1], p[3]); // B D
			QCD = scalar_triple(ray_dir, p[2], p[3]); // C D

			sQAB = copysignf(1.0f, QAB); // A B
			sQBC = copysignf(1.0f, QBC); // B C
			sQAC = copysignf(1.0f, QAC); // A C
			sQAD = copysignf(1.0f, QAD); // A D
			sQBD = copysignf(1.0f, QBD); // B D
			sQCD = copysignf(1.0f, QCD); // C D

			// ABC
			if ((sQAB != 0 && sQAC != 0 && sQBC != 0) &&
				(sQAB < 0 && sQAC > 0 && sQBC < 0))
			{
				outIdx = 3;
			}
			// BAD
			if ((sQAB != 0 && sQAD != 0 && sQBD != 0) &&
				(sQAB > 0 && sQAD < 0 && sQBD > 0))
			{
				outIdx = 2;
			}
			// CDA
			if ((sQAD != 0 && sQAC != 0 && sQCD != 0) &&
				(sQAD > 0 && sQAC < 0 && sQCD < 0))
			{
				outIdx = 1;
			}
			// DCB
			if ((sQBC != 0 && sQBD != 0 && sQCD != 0) &&
				(sQBC > 0 && sQBD < 0 && sQCD > 0))
			{
				outIdx = 0;
			}
			hit_cons_face = tets[index].face_cons[outIdx];
			index = tets[index].n[outIdx];

			counter++;
		}

		if (hit_cons_face)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray_origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray_dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray_dir, q));

			output[outputindex].position = ray_origin + f * glm::dot(e2, q) * ray_dir;
			output[outputindex].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[outputindex].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[outputindex].tet_idx = cons_faces[index].tet_idx;
			output[outputindex].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[outputindex].hit = true;
		}
		else
			output[outputindex].hit = false;
	}
}

//----------------------------------------- For TetMesh32 -------------------------------------------------

__global__
void ray_cast_kernel(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int offset, int tile_size,
	glm::vec3* points, TetMesh32::Tet32* tets, ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < resolution.x * resolution.y)
	{
		glm::vec3 ray_origin, ray_dir;
		int outputindex = init_ray(scene, resolution, tile_size, idx, ray_origin, ray_dir);

		unsigned int id[4];
		glm::vec2 p[4];

		const float sign = copysignf(1.0f, ray_dir.z);

		const float a = -1.0f / (sign + ray_dir.z);
		const float b = ray_dir.x * ray_dir.y * a;

		const glm::vec3 right(1.0f + sign * ray_dir.x * ray_dir.x * a, sign * b, -sign * ray_dir.x);
		const glm::vec3 up(b, sign + ray_dir.y * ray_dir.y * a, -ray_dir.y);

		for (int j = 0; j < 4; j++)
		{
			id[j] = source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray_origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		signed short outIdx = -1;

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
			outIdx = 0;
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
			outIdx = 1;
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
			outIdx = 2;
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			outIdx = 3;
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);
		}
		else
		{
			output[outputindex].hit = false;
			return;
		}

		int index = source_tet.n[outIdx];

		while (index >= 0)
		{
			id[outIdx] = id[3];
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray_origin;

			p[outIdx] = p[3];
			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			//p[3] = basis.project(newPoint);

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
					outIdx = 1;
				else
					outIdx = 0;
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
				outIdx = 2;
			else
				outIdx = 0;

			//prev_index = index;

			if (id[outIdx] == tets[index].v[0])
				index = tets[index].n[0];
			else if (id[outIdx] == tets[index].v[1])
				index = tets[index].n[1];
			else if (id[outIdx] == tets[index].v[2])
				index = tets[index].n[2];
			else
				index = tets[index].n[3];
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray_origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray_dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray_dir, q));

			output[outputindex].position = ray_origin + f * glm::dot(e2, q) * ray_dir;
			output[outputindex].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[outputindex].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[outputindex].tet_idx = cons_faces[index].tet_idx;
			output[outputindex].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[outputindex].hit = true;
		}
		else
			output[outputindex].hit = false;
	}
}

//----------------------------------------- For TetMesh20 -------------------------------------------------

__global__
void ray_cast_kernel(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int offset, int tile_size,
	glm::vec3* points, TetMesh20::Tet20* tets, ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < resolution.x * resolution.y)
	{
		glm::vec3 ray_origin, ray_dir;
		int outputindex = init_ray(scene, resolution, tile_size, idx, ray_origin, ray_dir);

		unsigned int id[4];
		glm::vec2 p[4];

		//int prev_index;
		signed short outIdx = -1;

		const float sign = copysignf(1.0f, ray_dir.z);

		const float a = -1.0f / (sign + ray_dir.z);
		const float b = ray_dir.x * ray_dir.y * a;

		const glm::vec3 right(1.0f + sign * ray_dir.x * ray_dir.x * a, sign * b, -sign * ray_dir.x);
		const glm::vec3 up(b, sign + ray_dir.y * ray_dir.y * a, -ray_dir.y);
		const glm::vec3 origin(ray_origin.x, ray_origin.y, ray_origin.z);

		const int mask = 0xE1;

		for (int j = 0; j < 4; j++)
		{
			id[j] = source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray_origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
			outIdx = 0;
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
			outIdx = 1;
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
			outIdx = 2;
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			outIdx = 3;
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);
		}
		else
		{
			output[outputindex].hit = false;
			return;
		}

		int index = source_tet.n[outIdx];
		id[outIdx] = id[3];
		p[outIdx] = p[3];

		while (index >= 0)
		{
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray_origin;

			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			//p[3] = basis.project(newPoint);

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
				{
					index = tets[index].n[(id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3])];
					//outIdx = 1;
					id[1] = id[3];
					p[1] = p[3];
				}
				else
				{
					index = tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
					//outIdx = 0;
					id[0] = id[3];
					p[0] = p[3];
				}
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
			{
				index = tets[index].n[(id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3])];
				//outIdx = 2;
				id[2] = id[3];
				p[2] = p[3];
			}
			else
			{
				index = tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
				//outIdx = 0;
				id[0] = id[3];
				p[0] = p[3];
			}

			//prev_index = index;

			//int idx = 0;
			//for (int i = 0; i < 4; ++i)
			//{
			//    if (id[outIdx] > id[i])
			//        idx++;
			//}

			//index = m_tet20s[index].n[idx];
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray_origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray_dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray_dir, q));
			output[outputindex].position = ray_origin + f * glm::dot(e2, q) * ray_dir;
			output[outputindex].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[outputindex].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[outputindex].tet_idx = cons_faces[index].tet_idx;
			output[outputindex].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[outputindex].hit = true;
		}
		else
			output[outputindex].hit = false;
	}
}

//---------------------------------------- For TetMesh16 --------------------------------------------------

__global__
void ray_cast_kernel(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int offset, int tile_size,
	glm::vec3* points, TetMesh16::Tet16* tets, ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < resolution.x * resolution.y)
	{
		glm::vec3 ray_origin, ray_dir;
		int outputindex = init_ray(scene, resolution, tile_size, idx, ray_origin, ray_dir);

		unsigned int id[4];
		glm::vec2 p[4];

		const float sign = copysignf(1.0f, ray_dir.z);

		const float a = -1.0f / (sign + ray_dir.z);
		const float b = ray_dir.x * ray_dir.y * a;

		const glm::vec3 right(1.0f + sign * ray_dir.x * ray_dir.x * a, sign * b, -sign * ray_dir.x);
		const glm::vec3 up(b, sign + ray_dir.y * ray_dir.y * a, -ray_dir.y);

		int index;

		for (int j = 0; j < 4; j++)
		{
			id[j] = source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray_origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
		{
			index = source_tet.n[0];
			id[0] = id[3];
			p[0] = p[3];
		}
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
		{
			index = source_tet.n[1];
			id[1] = id[3];
			p[1] = p[3];
		}
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
		{
			index = source_tet.n[2];
			id[2] = id[3];
			p[2] = p[3];
		}
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);

			index = source_tet.n[3];
		}
		else
		{
			output[outputindex].hit = false;
			return;
		}

		int nx = source_tet.idx;

		while (index >= 0)
		{
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray_origin;

			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			const int idx = (id[3] > id[0]) + (id[3] > id[1]) + (id[3] > id[2]);

			if (idx != 0)
				nx ^= tets[index].n[idx - 1];

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
				{
					const int idx2 = (id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3]);

					if (idx2 != 0)
						nx ^= tets[index].n[idx2 - 1];

					id[1] = id[3];
					p[1] = p[3];
				}
				else
				{
					const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

					if (idx2 != 0)
						nx ^= tets[index].n[idx2 - 1];

					id[0] = id[3];
					p[0] = p[3];
				}
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
			{
				const int idx2 = (id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3]);

				if (idx2 != 0)
					nx ^= tets[index].n[idx2 - 1];

				id[2] = id[3];
				p[2] = p[3];
			}
			else
			{
				const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

				if (idx2 != 0)
					nx ^= tets[index].n[idx2 - 1];

				id[0] = id[3];
				p[0] = p[3];
			}

			swap(nx, index);
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray_origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray_dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray_dir, q));
			output[outputindex].position = ray_origin + f * glm::dot(e2, q) * ray_dir;
			output[outputindex].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[outputindex].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[outputindex].tet_idx = cons_faces[index].tet_idx;
			output[outputindex].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[outputindex].hit = true;
		}
		else
			output[outputindex].hit = false;
	}
}

//---------------------------------------- For TetMesh96 --------------------------------------------------

__global__
void ray_cast_kernel(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int offset, int tile_size,
	/*cudaTextureObject_t t_vertices,*/ float4* d_vertices, cudaTextureObject_t t_tet96s, int2* d_tet96s, ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int idx = offset + blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < resolution.x * resolution.y)
	{
		glm::vec3 ray_origin, ray_dir;
		int outputindex = init_ray(scene, resolution, tile_size, idx, ray_origin, ray_dir);

		TetMesh80::Plucker pl_ray = make_plucker_from_ray(ray_origin, ray_dir);

		int id_tetra, id_vertex, id_entry_face;
		int id_exit;
		int semantic;
		int2 tetra;

		// Get first tetra 
		id_tetra = source_tet.idx * 4;

		tetra = LOAD_TETRA(id_tetra);

		glm::vec3 a(((float4)LOAD_VERTEX(id_tetra + 2)).x, ((float4)LOAD_VERTEX(id_tetra + 2)).y, ((float4)LOAD_VERTEX(id_tetra + 2)).z);
		glm::vec3 b(((float4)LOAD_VERTEX(id_tetra + 3)).x, ((float4)LOAD_VERTEX(id_tetra + 3)).y, ((float4)LOAD_VERTEX(id_tetra + 3)).z);
		glm::vec3 dir  = glm::normalize(b - a);
		glm::vec3 vv = glm::cross(a, b);

		id_entry_face = (dir[0] * pl_ray.m_Pi[3] +
			dir[1] * pl_ray.m_Pi[4] +
			dir[2] * pl_ray.m_Pi[5] +
			vv[0] * pl_ray.m_Pi[0] +
			vv[1] * pl_ray.m_Pi[1] +
			vv[2] * pl_ray.m_Pi[2]) < 0.0f;

		//============================================================== 
		// Now begin traversal 
		//============================================================== 
		int cpt = 0;
		do {
			if (cpt++ == 5000) { // In case of numeric error 
				semantic = TCDT_MAGIC_ERROR;
				break;
			}

			id_exit = get_exit_face(pl_ray, id_tetra, id_entry_face, d_vertices/*t_vertices*/);

			tetra = LOAD_TETRA(id_tetra + id_exit);
			semantic = tetra.y;

			if (semantic >= 0)
				break;

			// Update data for next tetra 
			id_tetra = GET_NEXT_TETRA(tetra.x) * 4;
			id_entry_face = GET_NEXT_FACE(tetra.x);

		} while (true);
		// Store result 
		//out.m_idFace = idVertex + idExit; 
		//out.m_idMtl = semantic; 
		//out.m_idVol = idTetra; 

		if (semantic >= 1)
			output[outputindex].hit = true;
		else
			output[outputindex].hit = false;

	}
}

//============================ Ray Traversal Kernels with Rays from CPU ===================================

//----------------------------------------- For TetMesh32 -------------------------------------------------

__global__
void ray_traversal_kernel(Ray* rays, int rays_size, int offset, glm::vec3* points, TetMesh32::Tet32* tets,
	ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
	if (i < rays_size)
	{
		Ray ray = rays[i];
		unsigned int id[4];
		glm::vec2 p[4];

		const float sign = copysignf(1.0f, ray.dir.z);

		const float a = -1.0f / (sign + ray.dir.z);
		const float b = ray.dir.x * ray.dir.y * a;

		const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
		const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);

		for (int j = 0; j < 4; j++)
		{
			id[j] = rays[i].source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray.origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		signed short outIdx = -1;

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
			outIdx = 0;
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
			outIdx = 1;
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
			outIdx = 2;
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			outIdx = 3;
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);
		}
		else
		{
			output[i].hit = false;
			return;
		}

		int index = ray.source_tet.n[outIdx];

		while (index >= 0)
		{
			id[outIdx] = id[3];
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray.origin;

			p[outIdx] = p[3];
			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			//p[3] = basis.project(newPoint);

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
					outIdx = 1;
				else
					outIdx = 0;
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
				outIdx = 2;
			else
				outIdx = 0;

			//prev_index = index;

			if (id[outIdx] == tets[index].v[0])
				index = tets[index].n[0];
			else if (id[outIdx] == tets[index].v[1])
				index = tets[index].n[1];
			else if (id[outIdx] == tets[index].v[2])
				index = tets[index].n[2];
			else
				index = tets[index].n[3];
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray.origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray.dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));

			output[i].position = ray.origin + f * glm::dot(e2, q) * ray.dir;//***
			output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];//***
			output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0]; //***
			output[i].tet_idx = cons_faces[index].tet_idx;
			output[i].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[i].hit = true;
		}
		else
			output[i].hit = false;
	}
}

//----------------------------------------- For TetMesh20 -------------------------------------------------

__global__
void ray_traversal_kernel(Ray* rays, int rays_size, int offset, glm::vec3* points, TetMesh20::Tet20* tets,
	ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
	if (i < rays_size)
	{
		Ray ray = rays[i];
		unsigned int id[4];
		glm::vec2 p[4];

		//int prev_index;
		signed short outIdx = -1;

		const float sign = copysignf(1.0f, ray.dir.z);

		const float a = -1.0f / (sign + ray.dir.z);
		const float b = ray.dir.x * ray.dir.y * a;

		const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
		const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);
		const glm::vec3 origin(ray.origin.x, ray.origin.y, ray.origin.z);

		const int mask = 0xE1;

		for (int j = 0; j < 4; j++)
		{
			id[j] = ray.source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray.origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
			outIdx = 0;
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
			outIdx = 1;
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
			outIdx = 2;
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			outIdx = 3;
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);
		}
		else
		{
			output[i].hit = false;
			return;
		}

		int index = ray.source_tet.n[outIdx];
		id[outIdx] = id[3];
		p[outIdx] = p[3];

		while (index >= 0)
		{
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray.origin;

			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			//p[3] = basis.project(newPoint);

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
				{
					index = tets[index].n[(id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3])];
					//outIdx = 1;
					id[1] = id[3];
					p[1] = p[3];
				}
				else
				{
					index = tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
					//outIdx = 0;
					id[0] = id[3];
					p[0] = p[3];
				}
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
			{
				index = tets[index].n[(id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3])];
				//outIdx = 2;
				id[2] = id[3];
				p[2] = p[3];
			}
			else
			{
				index = tets[index].n[(id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3])];
				//outIdx = 0;
				id[0] = id[3];
				p[0] = p[3];
			}

			//prev_index = index;

			//int idx = 0;
			//for (int i = 0; i < 4; ++i)
			//{
			//    if (id[outIdx] > id[i])
			//        idx++;
			//}

			//index = m_tet20s[index].n[idx];
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray.origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray.dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
			output[i].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
			output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[i].tet_idx = cons_faces[index].tet_idx;
			output[i].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[i].hit = true;
		}
		else
			output[i].hit = false;
	}
}

//--------------------------------------- For TetMesh16 -----------------------------------------------

__global__
void ray_traversal_kernel(Ray* rays, int rays_size, int offset, glm::vec3* points, TetMesh16::Tet16* tets,
	ConstrainedFace* cons_faces, Face* faces, IntersectionData* output)
{
	int i = offset + blockIdx.x * blockDim.x + threadIdx.x;
	if (i < rays_size)
	{
		Ray ray = rays[i];
		unsigned int id[4];
		glm::vec2 p[4];

		const float sign = copysignf(1.0f, ray.dir.z);

		const float a = -1.0f / (sign + ray.dir.z);
		const float b = ray.dir.x * ray.dir.y * a;

		const glm::vec3 right(1.0f + sign * ray.dir.x * ray.dir.x * a, sign * b, -sign * ray.dir.x);
		const glm::vec3 up(b, sign + ray.dir.y * ray.dir.y * a, -ray.dir.y);

		int index;

		for (int j = 0; j < 4; j++)
		{
			id[j] = ray.source_tet.v[j];
			const glm::vec3 point = points[id[j]] - ray.origin;
			p[j].x = glm::dot(right, point);
			p[j].y = glm::dot(up, point);
		}

		if (p[2].x * p[1].y <= p[2].y * p[1].x && p[1].x * p[3].y <= p[1].y * p[3].x && p[3].x * p[2].y <= p[3].y * p[2].x)
		{
			index = ray.source_tet.n[0];
			id[0] = id[3];
			p[0] = p[3];
		}
		else if (p[2].x * p[3].y <= p[2].y * p[3].x && p[3].x * p[0].y <= p[3].y * p[0].x && p[0].x * p[2].y <= p[0].y * p[2].x)
		{
			index = ray.source_tet.n[1];
			id[1] = id[3];
			p[1] = p[3];
		}
		else if (p[0].x * p[3].y <= p[0].y * p[3].x && p[3].x * p[1].y <= p[3].y * p[1].x && p[1].x * p[0].y <= p[1].y * p[0].x)
		{
			index = ray.source_tet.n[2];
			id[2] = id[3];
			p[2] = p[3];
		}
		else if (p[0].x * p[1].y <= p[0].y * p[1].x && p[1].x * p[2].y <= p[1].y * p[2].x && p[2].x * p[0].y <= p[2].y * p[0].x)
		{
			swap(id[0], id[1]);
			swapvec2(p[0], p[1]);

			index = ray.source_tet.n[3];
		}
		else
		{
			output[i].hit = false;
			return;
		}

		int nx = ray.source_tet.idx;

		while (index >= 0)
		{
			id[3] = tets[index].x ^ id[0] ^ id[1] ^ id[2];
			const glm::vec3 newPoint = points[id[3]] - ray.origin;

			p[3].x = glm::dot(right, newPoint);
			p[3].y = glm::dot(up, newPoint);

			const int idx = (id[3] > id[0]) + (id[3] > id[1]) + (id[3] > id[2]);

			if (idx != 0)
				nx ^= tets[index].n[idx - 1];

			if (p[3].x * p[0].y < p[3].y * p[0].x) // copysignf here?
			{
				if (p[3].x * p[2].y >= p[3].y * p[2].x)
				{
					const int idx2 = (id[1] > id[0]) + (id[1] > id[2]) + (id[1] > id[3]);

					if (idx2 != 0)
						nx ^= tets[index].n[idx2 - 1];

					id[1] = id[3];
					p[1] = p[3];
				}
				else
				{
					const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

					if (idx2 != 0)
						nx ^= tets[index].n[idx2 - 1];

					id[0] = id[3];
					p[0] = p[3];
				}
			}
			else if (p[3].x * p[1].y < p[3].y * p[1].x)
			{
				const int idx2 = (id[2] > id[0]) + (id[2] > id[1]) + (id[2] > id[3]);

				if (idx2 != 0)
					nx ^= tets[index].n[idx2 - 1];

				id[2] = id[3];
				p[2] = p[3];
			}
			else
			{
				const int idx2 = (id[0] > id[1]) + (id[0] > id[2]) + (id[0] > id[3]);

				if (idx2 != 0)
					nx ^= tets[index].n[idx2 - 1];

				id[0] = id[3];
				p[0] = p[3];
			}

			swap(nx, index);
		}

		if (index != -1)
		{
			index = (index & 0x7FFFFFFF);
			const Face& face = faces[cons_faces[index].face_idx];

			const glm::vec3* v = face.vertices;
			const glm::vec3* n = face.normals;
			const glm::vec2* t = face.uvs;

			const glm::vec3 e1 = v[1] - v[0];
			const glm::vec3 e2 = v[2] - v[0];
			const glm::vec3 s = ray.origin - v[0];
			const glm::vec3 q = glm::cross(s, e1);
			const glm::vec3 p = glm::cross(ray.dir, e2);
			const float f = 1.0f / glm::dot(e1, p);
			const glm::vec2 bary(f * glm::dot(s, p), f * glm::dot(ray.dir, q));
			output[i].position = ray.origin + f * glm::dot(e2, q) * ray.dir;
			output[i].normal = bary.x * n[1] + bary.y * n[2] + (1 - bary.x - bary.y) * n[0];
			output[i].uv = bary.x * t[1] + bary.y * t[2] + (1 - bary.x - bary.y) * t[0];
			output[i].tet_idx = cons_faces[index].tet_idx;
			output[i].neighbor_tet_idx = cons_faces[index].other_tet_idx;

			output[i].hit = true;
		}
		else
			output[i].hit = false;
	}
}

//==================================== Copies tetmesh to device memory ======================================

void copy_to_gpu_helper(TetMesh& tet_mesh)
{
	cudaFree(d_points);
	cudaMalloc(&d_points, tet_mesh.m_points.size() * sizeof(glm::vec3));
	cudaMemcpy(d_points, tet_mesh.m_points.data(), tet_mesh.m_points.size() * sizeof(glm::vec3), cudaMemcpyHostToDevice);

	cudaFree(d_cons_faces);
	cudaMalloc(&d_cons_faces, tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace));
	cudaMemcpy(d_cons_faces, tet_mesh.m_constrained_faces.data(), tet_mesh.m_constrained_faces.size() * sizeof(ConstrainedFace), cudaMemcpyHostToDevice);

	cudaFree(d_faces);
	cudaMalloc(&d_faces, tet_mesh.faces.size() * sizeof(Face));
	cudaMemcpy(d_faces, tet_mesh.faces.data(), tet_mesh.faces.size() * sizeof(Face), cudaMemcpyHostToDevice);

	//print_cuda_error("CUDA copy error");
}

//-------------------------------- Methods to copy specific components -------------------------------------

void copy_to_gpu(TetMesh32& tet_mesh)
{
	copy_to_gpu_helper(tet_mesh);

	cudaFree(d_tet32s);
	cudaMalloc(&d_tet32s, tet_mesh.m_tets.size() * sizeof(TetMesh32::Tet32));
	cudaMemcpy(d_tet32s, tet_mesh.m_tet32s, tet_mesh.m_tets.size() * sizeof(TetMesh32::Tet32), cudaMemcpyHostToDevice);

	print_cuda_error("CUDA copy Tet32 to GPU");
}

void copy_to_gpu(TetMesh20& tet_mesh)
{
	copy_to_gpu_helper(tet_mesh);

	cudaFree(d_tet20s);
	cudaMalloc(&d_tet20s, tet_mesh.m_tets.size() * sizeof(TetMesh20::Tet20));
	cudaMemcpy(d_tet20s, tet_mesh.m_tet20s.data(), tet_mesh.m_tets.size() * sizeof(TetMesh20::Tet20), cudaMemcpyHostToDevice);

	print_cuda_error("CUDA copy Tet20 to GPU");
}

void copy_to_gpu(TetMesh16& tet_mesh)
{
	copy_to_gpu_helper(tet_mesh);

	cudaFree(d_tet16s);
	cudaMalloc(&d_tet16s, tet_mesh.m_tets.size() * sizeof(TetMesh16::Tet16));
	cudaMemcpy(d_tet16s, tet_mesh.m_tet16s, tet_mesh.m_tets.size() * sizeof(TetMesh16::Tet16), cudaMemcpyHostToDevice);

	print_cuda_error("CUDA copy Tet16 to GPU");
}

void copy_to_gpu(TetMeshSctp& tet_mesh)
{
	copy_to_gpu_helper(tet_mesh);

	cudaFree(d_tetSctps);
	cudaMalloc(&d_tetSctps, tet_mesh.m_tets.size() * sizeof(TetMeshSctp::TetSctp));
	cudaMemcpy(d_tetSctps, tet_mesh.m_tet_sctps, tet_mesh.m_tets.size() * sizeof(TetMeshSctp::TetSctp), cudaMemcpyHostToDevice);

	print_cuda_error("CUDA copy TetScTP to GPU");
}

void copy_to_gpu(TetMesh80& tet_mesh)
{
	copy_to_gpu_helper(tet_mesh);

	num_int2_d_tets = tet_mesh.m_tets.size() * 4;
	num_float4_d_vertices = tet_mesh.m_tets.size() * 4;

	int2* h_tets = new int2[num_int2_d_tets];
	float4* h_vertices = new float4[num_float4_d_vertices];

	for (int i = 0; i < tet_mesh.m_tets.size(); ++i) {
		const TetMesh80::Tet80& tetra = tet_mesh.m_tet80s[i];
		const int idTetra = i * 4;

		for (int j = 0; j < 4; ++j) {
			h_tets[idTetra + j] = make_int2(tetra.m_nextTetraFace[j],
				tetra.m_semantics[j]
			);
		}
	}
	for (int i = 0; i < tet_mesh.m_tets.size()*4; ++i) {
		const glm::vec4& v = tet_mesh.m_vertices[i];
		h_vertices[i] = make_float4(v.x, v.y, v.z, v.w);
	}

	const size_t size_tets = num_int2_d_tets * sizeof(int2);
	const size_t size_vertices = num_float4_d_vertices * sizeof(float4);

	check_cuda(cudaMalloc(&d_tet96s, size_tets));
	check_cuda(cudaMemcpy(d_tet96s, h_tets, size_tets, cudaMemcpyHostToDevice));
	check_cuda(cudaMalloc(&d_vertices, size_vertices));
	check_cuda(cudaMemcpy(d_vertices, h_vertices, size_vertices, cudaMemcpyHostToDevice));

#ifdef TCDT_CUDA_TCDT_USE_TEXTURES_OBJECTS
	{
		// Create texture object
		cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(resDesc));
		resDesc.resType = cudaResourceTypeLinear;
		resDesc.res.linear.devPtr = d_tet96s;
		resDesc.res.linear.desc = cudaCreateChannelDesc<int2>();
		resDesc.res.linear.sizeInBytes = size_tets;

		cudaTextureDesc texDesc;
		memset(&texDesc, 0, sizeof(texDesc));
		texDesc.readMode = cudaReadModeElementType;

		check_cuda(cudaCreateTextureObject(&t_tet96s, &resDesc,
			&texDesc, NULL));
	}
	{
		// Create texture object
		cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(resDesc));
		resDesc.resType = cudaResourceTypeLinear;
		resDesc.res.linear.devPtr = d_vertices;
		resDesc.res.linear.desc = cudaCreateChannelDesc<float4>();
		resDesc.res.linear.sizeInBytes = size_vertices;

		cudaTextureDesc texDesc;
		memset(&texDesc, 0, sizeof(texDesc));
		texDesc.readMode = cudaReadModeElementType;

		check_cuda(cudaCreateTextureObject(&t_vertices, &resDesc,
			&texDesc, NULL));
	}
#endif
	delete[] h_tets;
	delete[] h_vertices;
	print_cuda_error("CUDA copy Tet80 as Tet96 to GPU");
}

//================================= Traversal of rays initialized at CPU =================================
void traverse_rays_gpu(Ray* rays, unsigned int rays_size, unsigned int tet_mesh_type, IntersectionData* output)
{
	// Allocate space for device copy of data
	if (old_size != rays_size)
	{
		cudaFree(d_rays);
		cudaFree(d_intersectdata);
		cudaMalloc(&d_rays, rays_size * sizeof(Ray));
		cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
		old_size = rays_size;
	}

	unsigned int stream_size = rays_size / NSTREAMS;
	int stream_bytes = stream_size * sizeof(Ray);

	float kernel_time = 0;

	std::chrono::steady_clock::time_point start, end;

	// Launch kernel on GPU
	int t = 512;
	start = std::chrono::steady_clock::now();
	cudaMemcpy(d_rays, rays, rays_size * sizeof(Ray), cudaMemcpyHostToDevice);
	end = std::chrono::steady_clock::now();
	Stats::gpu_copy_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;

	start = std::chrono::steady_clock::now();
	if (tet_mesh_type == 0)
	{
		//raycast_kernel <<< stream_size / t, t,  0, streams[i]>>> (d_rays, rays_size, offset, d_points, d_tet32s, d_cons_faces, d_faces, d_intersectdata);
		ray_traversal_kernel << < rays_size / t, t >> > (d_rays, rays_size, 0, d_points, d_tet32s, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 1)
	{
		ray_traversal_kernel << < rays_size / t, t >> > (d_rays, rays_size, 0, d_points, d_tet20s, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 2)
	{
		ray_traversal_kernel << < rays_size / t, t >> > (d_rays, rays_size, 0, d_points, d_tet16s, d_cons_faces, d_faces, d_intersectdata);
	}
	cudaDeviceSynchronize();

	end = std::chrono::steady_clock::now();
	kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
	Stats::gpu_kernel_time = kernel_time;

	start = std::chrono::steady_clock::now();
	cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
	end = std::chrono::steady_clock::now();
	Stats::gpu_copy_back_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
	//print_cuda_error("copyback");

}

//------------------------------------- Asynch ray traversal ---------------------------------------------
void traverse_rays_gpu(Ray* rays, unsigned int rays_size, int num_streams, unsigned int tet_mesh_type, int id_s, IntersectionData* output)
{
	if (!streams)
	{
		streams = new cudaStream_t[num_streams];
	}
	// Allocate space for device copy of data
	if (old_size != rays_size)
	{
		old_size = rays_size;
		cudaFree(d_rays);
		cudaFree(d_intersectdata);
		cudaMalloc(&d_rays, rays_size * sizeof(Ray));
		cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
	}
	cudaStreamCreate(&streams[id_s]);
	unsigned int stream_size = rays_size / num_streams;
	int stream_bytes = stream_size * sizeof(Ray);

	int t = 512;
	int offset = id_s * stream_size;

	if (streams)
	{
		cudaMemcpyAsync(&d_rays[offset], &rays[offset], stream_bytes, cudaMemcpyHostToDevice, streams[id_s]);

		if (tet_mesh_type == 0)
		{
			ray_traversal_kernel << < stream_size / t, t, 0, streams[id_s] >> > (d_rays, rays_size, offset, d_points, d_tet32s, d_cons_faces, d_faces, d_intersectdata);
		}
		else if (tet_mesh_type == 1)
		{
			ray_traversal_kernel << < stream_size / t, t, 0, streams[id_s] >> > (d_rays, rays_size, offset, d_points, d_tet20s, d_cons_faces, d_faces, d_intersectdata);
		}
		else if (tet_mesh_type == 2)
		{
			ray_traversal_kernel << < stream_size / t, t, 0, streams[id_s] >> > (d_rays, rays_size, offset, d_points, d_tet16s, d_cons_faces, d_faces, d_intersectdata);
		}

		offset = id_s * stream_size;
		cudaMemcpyAsync(&output[offset], &d_intersectdata[offset], stream_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost, streams[id_s]);
		cudaStreamDestroy(streams[id_s]);
	}
}

//============================== Initialization and traversal of rays in GPU =========================================

void cast_rays_gpu(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int tile_size, unsigned int tet_mesh_type, IntersectionData* output)
{
	unsigned int rays_size = resolution.x * resolution.y;
	// Allocate space for device copy of data
	if (old_size != rays_size)
	{
		cudaFree(d_rays);
		cudaFree(d_intersectdata);
		cudaFree(d_res);
		cudaMalloc(&d_rays, rays_size * sizeof(Ray));
		cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
		cudaMalloc(&d_res, sizeof(glm::ivec2));
		old_size = rays_size;

		cudaFree(d_source_tet);
		cudaFree(d_scene);
		cudaMalloc(&d_source_tet, sizeof(SourceTet));
		cudaMalloc(&d_scene, sizeof(Scene));
		cudaMemcpy(d_source_tet, new SourceTet(source_tet), sizeof(SourceTet), cudaMemcpyHostToDevice);
		cudaMemcpy(d_scene, new Scene(scene), sizeof(Scene), cudaMemcpyHostToDevice);
		cudaMemcpy(d_res, new glm::ivec2(resolution), sizeof(glm::ivec2), cudaMemcpyHostToDevice);
	}

	if (scene.camTarget != old_target || scene.camDist != old_dist ||
		scene.camOrbitX != old_orbit_x || scene.camOrbitY != old_orbit_y)
	{
		cudaFree(d_source_tet);
		cudaFree(d_scene);
		cudaMalloc(&d_source_tet, sizeof(SourceTet));
		cudaMalloc(&d_scene, sizeof(Scene));
		old_target = scene.camTarget;
		old_dist = scene.camDist;
		old_orbit_x = scene.camOrbitX;
		old_orbit_y = scene.camOrbitY;

	}
	cudaMemcpy(d_source_tet, new SourceTet(source_tet), sizeof(SourceTet), cudaMemcpyHostToDevice);
	cudaMemcpy(d_scene, new Scene(scene), sizeof(Scene), cudaMemcpyHostToDevice);

	unsigned int stream_size = rays_size / NSTREAMS;
	int stream_bytes = stream_size * sizeof(Ray);

	float kernel_time = 0;
	Stats::gpu_copy_time = 0.0f;

	std::chrono::steady_clock::time_point start, end;

	// Launch kernel on GPU
	int t = 512;

	start = std::chrono::steady_clock::now();
	if (tet_mesh_type == 0)
	{
		ray_cast_kernel << < rays_size / t, t >> > (*d_scene, *d_source_tet, *d_res, 0, tile_size, d_points, d_tet32s, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 1)
	{
		ray_cast_kernel << < rays_size / t, t >> > (*d_scene, *d_source_tet, *d_res, 0, tile_size, d_points, d_tet20s, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 2)
	{
		ray_cast_kernel << < rays_size / t, t >> > (*d_scene, *d_source_tet, *d_res, 0, tile_size, d_points, d_tet16s, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 3)
	{
		ray_cast_kernel << < rays_size / t, t >> > (*d_scene, *d_source_tet, *d_res, 0, tile_size, d_points, d_tetSctps, d_cons_faces, d_faces, d_intersectdata);
	}
	else if (tet_mesh_type == 4)
	{
		ray_cast_kernel << < rays_size / t, t >> > (*d_scene, *d_source_tet, *d_res, 0, tile_size, /*t_vertices,*/ d_vertices, t_tet96s, d_tet96s, d_cons_faces, d_faces, d_intersectdata);
	}
	cudaDeviceSynchronize();

	end = std::chrono::steady_clock::now();
	kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
	Stats::gpu_kernel_time = kernel_time;

	start = std::chrono::steady_clock::now();
	cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
	end = std::chrono::steady_clock::now();
	Stats::gpu_copy_back_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
}

//---------------------------------- Asynch ray casting ------------------------------------------

void cast_rays_gpu(Scene& scene, SourceTet& source_tet, glm::ivec2& resolution, int tile_size, int num_streams, int id_s, unsigned int tet_mesh_type, IntersectionData* output)
{
	if (!streams)
	{
		streams = new cudaStream_t[num_streams];
	}

	unsigned int rays_size = resolution.x * resolution.y;
	// Allocate space for device copy of data
	if (old_size != rays_size && id_s == 0)
	{
		//cudaFree(d_rays);
		//cudaFree(d_intersectdata);
		//cudaFree(d_res);
		cudaMalloc(&d_rays, rays_size * sizeof(Ray));
		cudaMalloc(&d_intersectdata, rays_size * sizeof(IntersectionData));
		cudaMalloc(&d_res, sizeof(glm::ivec2));
		old_size = rays_size;

		//cudaFree(d_source_tet);
		//cudaFree(d_scene);
		cudaMalloc(&d_source_tet, sizeof(SourceTet));
		cudaMalloc(&d_scene, sizeof(Scene));
		cudaMemcpy(d_source_tet, new SourceTet(source_tet), sizeof(SourceTet), cudaMemcpyHostToDevice);
		cudaMemcpy(d_scene, new Scene(scene), sizeof(Scene), cudaMemcpyHostToDevice);
		cudaMemcpy(d_res, new glm::ivec2(resolution), sizeof(glm::ivec2), cudaMemcpyHostToDevice);
	}

	if ((scene.camTarget != old_target || scene.camDist != old_dist ||
		scene.camOrbitX != old_orbit_x || scene.camOrbitY != old_orbit_y) &&
		(id_s == 0))
	{
		//cudaFree(d_source_tet);
		//cudaFree(d_scene);
		cudaMalloc(&d_source_tet, sizeof(SourceTet));
		cudaMalloc(&d_scene, sizeof(Scene));
		old_target = scene.camTarget;
		old_dist = scene.camDist;
		old_orbit_x = scene.camOrbitX;
		old_orbit_y = scene.camOrbitY;
	}
	cudaStreamCreate(&streams[id_s]);
	unsigned int stream_size = rays_size / num_streams;
	int stream_bytes = stream_size * sizeof(Ray);

	int t = 512;
	int offset = id_s * stream_size;

	//std::chrono::steady_clock::time_point start, end;

	// Launch kernel on GPU

	//start = std::chrono::steady_clock::now();
	if (streams)
	{
		if (tet_mesh_type == 0)
		{
			ray_cast_kernel << < stream_size / t, t, 0, streams[id_s] >> > (*d_scene, *d_source_tet, *d_res, offset, tile_size, d_points, d_tet32s, d_cons_faces, d_faces, d_intersectdata);
		}
		else if (tet_mesh_type == 1)
		{
			ray_cast_kernel << < stream_size / t, t, 0, streams[id_s] >> > (*d_scene, *d_source_tet, *d_res, offset, tile_size, d_points, d_tet20s, d_cons_faces, d_faces, d_intersectdata);
		}
		else if (tet_mesh_type == 2)
		{
			ray_cast_kernel << < stream_size / t, t, 0, streams[id_s] >> > (*d_scene, *d_source_tet, *d_res, offset, tile_size, d_points, d_tet16s, d_cons_faces, d_faces, d_intersectdata);
		}
		else if (tet_mesh_type == 3)
		{
			ray_cast_kernel << < stream_size / t, t, 0, streams[id_s] >> > (*d_scene, *d_source_tet, *d_res, offset, tile_size, d_points, d_tetSctps, d_cons_faces, d_faces, d_intersectdata);
		}
		offset = id_s * stream_size;
		cudaMemcpyAsync(&output[offset], &d_intersectdata[offset], stream_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost, streams[id_s]);
		cudaStreamDestroy(streams[id_s]);
	}

	/*end = std::chrono::steady_clock::now();
	kernel_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;
	Stats::gpu_kernel_time = kernel_time;

	start = std::chrono::steady_clock::now();
	cudaMemcpy(output, d_intersectdata, rays_size * sizeof(IntersectionData), cudaMemcpyDeviceToHost);
	end = std::chrono::steady_clock::now();
	Stats::gpu_copy_back_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1e3;*/
}


