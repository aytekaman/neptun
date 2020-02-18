#include "bvh_embree.h"

namespace neptun
{

bool BvhEmbree::build(const Triangle* primitives, size_t primitive_count)
{
    RTCDevice rtc_device = rtcNewDevice("");
    m_rtc_scene = rtcNewScene(rtc_device);

    RTCGeometry mesh = rtcNewGeometry(rtc_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    glm::vec3* vertices = (glm::vec3*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(glm::vec3), primitive_count * 3);


    for (size_t i = 0; i < primitive_count; ++i)
    {
        vertices[3 * i + 0].x = primitives[i].v[0].x;
        vertices[3 * i + 0].y = primitives[i].v[0].y;
        vertices[3 * i + 0].z = primitives[i].v[0].z;

        vertices[3 * i + 1].x = primitives[i].v[1].x;
        vertices[3 * i + 1].y = primitives[i].v[1].y;
        vertices[3 * i + 1].z = primitives[i].v[1].z;

        vertices[3 * i + 2].x = primitives[i].v[2].x;
        vertices[3 * i + 2].y = primitives[i].v[2].y;
        vertices[3 * i + 2].z = primitives[i].v[2].z;
    }
    
    m_geometry_ids.resize(primitive_count);
    m_primitive_ids.resize(primitive_count);

    for (size_t i = 0; i < primitive_count; ++i)
    {
        m_geometry_ids[i] = primitives[i].geometry_id;
        m_primitive_ids[i] = primitives[i].primitive_id;
    }

    struct Tri
    {
        int v0;
        int v1;
        int v2;
    };

    Tri* triangles = (Tri*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Tri), primitive_count);

    for (int i = 0; i < primitive_count; ++i)
    {
        triangles[i].v0 = 3 * i + 0;
        triangles[i].v1 = 3 * i + 1;
        triangles[i].v2 = 3 * i + 2;
    }

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(m_rtc_scene, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(m_rtc_scene);
    return true;
}
    
void BvhEmbree::intersect1(RayHit& ray_hit) const 
{
    Ray& ray = ray_hit.ray;
    Hit& hit = ray_hit.hit;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRayHit rtc_hit;

    rtc_hit.ray.org_x = ray.org.x;
    rtc_hit.ray.org_y = ray.org.y;
    rtc_hit.ray.org_z = ray.org.z;

    rtc_hit.ray.dir_x = ray.dir.x;
    rtc_hit.ray.dir_y = ray.dir.y;
    rtc_hit.ray.dir_z = ray.dir.z;

    rtc_hit.ray.tnear = 0.0000001f;
    rtc_hit.ray.tfar = ray.max_t;

    rtc_hit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtc_hit.hit.primID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(m_rtc_scene, &context, &rtc_hit);

    if (rtc_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        hit.geometry_id = m_geometry_ids[rtc_hit.hit.geomID];
        hit.primitive_id = m_primitive_ids[rtc_hit.hit.primID];

        hit.bary.x = 1 - rtc_hit.hit.u - rtc_hit.hit.v;
        hit.bary.y = rtc_hit.hit.u;
        hit.n.x = rtc_hit.hit.Ng_x;
        hit.n.y = rtc_hit.hit.Ng_y;
        hit.n.z = rtc_hit.hit.Ng_z;
        ray.max_t = rtc_hit.ray.tfar;
    }
    else
    {
        hit.geometry_id = INVALID_GEOMETRY_ID;
        hit.primitive_id = INVALID_PRIMITIVE_ID;
    }
}
    
const char* BvhEmbree::name() const 
{
    return "Bvh Embree";
}
    
size_t BvhEmbree::get_size_in_bytes() const
{
    return 0;
}

}
