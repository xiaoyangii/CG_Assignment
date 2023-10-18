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
    // TO DO Use BVH.cpp's Intersect function instead of the current traversal method

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

Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Intersection intersection = intersect(ray); // 寻找光线与场景的交点

    if (!intersection.happened)                 // 如果没有交点，返回黑色（对像素没有贡献）
        return Vector3f(0, 0, 0);

    if (intersection.emit.norm() > 0) {         // 如果交点处的材质发光，直接返回其发射的颜色
        if (depth == 0) {
            return intersection.emit;
        }
        else {
            return Vector3f(0, 0, 0);           // 否则，终止路径
        }
    }

    // 从交点提取相关信息

    Vector3f& p = intersection.coords;
    Vector3f wo = normalize(-ray.direction);    // 从交点射出的方向
    Vector3f normal = normalize(intersection.normal);
    Material*& material = intersection.m;

    auto format = [](Vector3f& a) {             // 辅助函数，确保向量没有负分量
        if (a.x < 0) a.x = 0;
        if (a.y < 0) a.y = 0;
        if (a.z < 0) a.z = 0;
    };

    Vector3f L_direct;                          // 直接光照
    {
        Intersection inter_dir;                 // 采样光源以计算直接光照
        float pdf_dir;
        sampleLight(inter_dir, pdf_dir);

        Vector3f& x = inter_dir.coords;
        Vector3f ws = normalize(x - p);
        Vector3f light_normal = normalize(inter_dir.normal);

        auto pws = intersect(Ray(p, ws));       // 检查从交点到采样的光源是否被遮挡
        if (pws.happened && (pws.coords - x).norm() < 1e-2) {
            // 使用BRDF和阴影项计算直接光照贡献
            L_direct = inter_dir.emit * material->eval(ws, wo, normal) * dotProduct(normal, ws)
                * dotProduct(light_normal, -ws) / (dotProduct((x - p), (x - p)) * pdf_dir);
            // 确保结果是非负的
            format(L_direct);
        }
    }
    // 间接光照
    Vector3f L_indirect;
    {
        // 使用俄罗斯轮盘赌来决定是否继续路径追踪
        float RR = this->RussianRoulette;
        if (get_random_float() < RR) {
            // 根据材质的BRDF采样新的方向（wi）
            Vector3f wi = normalize(material->sample(wo, normal));
            L_indirect = castRay(Ray(p, wi), depth + 1)            // 递归地追踪采样的方向（wi）的新光线
                * material->eval(wi, wo, normal) * dotProduct(wi, normal)
                / (material->pdf(wi, wo, normal) * RR);
            format(L_indirect);                 // 确保结果是非负的
        }
    }

    // 返回直接光照和间接光照贡献之和
    return L_direct + L_indirect;
}