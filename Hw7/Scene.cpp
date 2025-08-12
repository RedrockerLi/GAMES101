//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f l_dir = Vector3f(0.f);

    Intersection first_intersection = intersect(ray);
    if (!first_intersection.happened) {
        return Vector3f(0.f);
    }

    if (first_intersection.m->hasEmission()) {
        return first_intersection.m->getEmission();
    }

    Intersection light_intersection;
    float light_pdf = 0;
    sampleLight(light_intersection, light_pdf);
    
    Vector3f point1 = first_intersection.coords;
    Vector3f N1 = first_intersection.normal;
    Vector3f point2 = light_intersection.coords;
    Vector3f N2 = light_intersection.normal;

    Vector3f dir1 = (point2 - point1).normalized();
    float dist1 = (point2 - point1).norm();

    Ray ws_ray(point1+dir1*EPSILON, dir1);
    Intersection light_break = intersect(ws_ray);
    if (light_break.distance - dist1 > -EPSILON){
        Vector3f emit = light_intersection.emit;
        l_dir = emit * first_intersection.m->eval(dir1, -ray.direction, N1) 
                * dotProduct( dir1, N1)
                * dotProduct(-dir1, N2) 
                / (light_pdf * dist1 * dist1);
    }

    return l_dir;
}