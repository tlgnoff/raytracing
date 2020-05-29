#include "Scene.h"
#include "Camera.h"
#include "Light.h"
#include "Material.h"
#include "Shape.h"
#include "tinyxml2.h"
#include "Image.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace tinyxml2;


Vector3f Scene::shading(Ray *ray, ReturnVal intersect_result, Shape *shape, Vector3f dir, int recursion_depth)
{
	size_t lightsSize = lights.size();
	size_t objectsSize = objects.size();


	Vector3f ambient_shading, diffuseRef, specularRef, ambientRef, mirrorRef, wo;
	Vector3f reflection_shading(0.f, 0.f, 0.f);
	Vector3f color;
	Vector3f wr = (-1 * dir + 2 * intersect_result.normal * intersect_result.normal.dot(dir)).normalized();
  Vector3f reflection_wo = (-1 * wr).normalized();

	Material* material;
	int phongExp;

	if(recursion_depth > maxRecursionDepth) {
		reflection_shading = backgroundColor;
		return reflection_shading;
	}

	material = materials[shape->matIndex-1];
	phongExp = material->phongExp;

	diffuseRef = material->diffuseRef;
	specularRef = material->specularRef;
	ambientRef = material->ambientRef;
	mirrorRef = material->mirrorRef;

	ambient_shading[0] = ambientRef[0] * ambientLight[0];
	ambient_shading[1] = ambientRef[1] * ambientLight[1];
	ambient_shading[2] = ambientRef[2] * ambientLight[2];
	color[0] = std::min(ambient_shading[0], 255.f);
	color[1] = std::min(ambient_shading[1], 255.f);
	color[2] = std::min(ambient_shading[2], 255.f);

	for(int j = 0; j < lightsSize; j++){
		PointLight *light = lights[j];
		Vector3f diffuse_shading, specular_shading, half_vector, wo, wi, shifted_origin, shiftedorgn1, shiftedorgn2, irradiance;
		Vector3f dir_to_light = light->position - intersect_result.coordinate;
		ReturnVal shadow_intResult;
		float costeta, cosalpha;


		shiftedorgn1 = intersect_result.coordinate - shadowRayEps*dir_to_light;
		shiftedorgn2 = intersect_result.coordinate + shadowRayEps*dir_to_light;
		shifted_origin = dir_to_light.dot(intersect_result.normal) < shadowRayEps ?  shiftedorgn1 : shiftedorgn2;

		if(mirrorRef.dot(mirrorRef) > 0.0){
				Ray *reflection_ray = new Ray(shifted_origin, wr);
				float tmin = std::numeric_limits<float>::max();
				ReturnVal reflection_intResult, tempintersect_result;
				Shape *reflection_obj = NULL;
				reflection_intResult.isIntersected = false;

				for(int k = 0; k < objectsSize; k++){
					Shape *tempobj = objects[k];
					tempintersect_result = tempobj->intersect(*reflection_ray);

					if(tempintersect_result.isIntersected && tempintersect_result.t < tmin+shadowRayEps && tempintersect_result.t > intTestEps){
						tmin = tempintersect_result.t;
						reflection_obj = tempobj;
						reflection_intResult = tempintersect_result;
					}
				}
				if(reflection_obj != NULL){
					reflection_shading = shading(reflection_ray, reflection_intResult, reflection_obj, reflection_wo, recursion_depth+1);
				}
		}

		Ray* shadow_ray = new Ray(shifted_origin, light->position - shifted_origin);

		for(int i = 0; i < objectsSize; i++){
			Shape *shadow_obj = objects[i];
			shadow_intResult = shadow_obj->intersect(*shadow_ray);
			if(shadow_intResult.isIntersected && shadow_intResult.t < shadow_ray->gett(light->position) && shadow_intResult.t > 0) {
				if(mirrorRef.dot(mirrorRef) > 0.0){
					color[0] = std::min(color[0] + mirrorRef[0]*reflection_shading[0], 255.f);
					color[1] = std::min(color[1] + mirrorRef[1]*reflection_shading[1], 255.f);
					color[2] = std::min(color[2] + mirrorRef[2]*reflection_shading[2], 255.f);
				}
				break;
			}
			shadow_intResult.isIntersected = false;
		}

		if(shadow_intResult.isIntersected) continue;

		wi = shadow_ray->direction.normalized();
		wo = (-1 * ray->direction).normalized();
		half_vector = (wi + wo).normalized();
		costeta = std::max(0.f, std::min(wi.dot(intersect_result.normal), 1.f));
		cosalpha = pow(std::max(0.f, std::min(intersect_result.normal.dot(half_vector), 1.f)), phongExp);
		irradiance = light->computeLightContribution(intersect_result.coordinate);

		diffuse_shading[0] = diffuseRef[0] * costeta * irradiance[0];
		diffuse_shading[1] = diffuseRef[1] * costeta * irradiance[1];
		diffuse_shading[2] = diffuseRef[2] * costeta * irradiance[2];
		specular_shading[0] = specularRef[0] * cosalpha * irradiance[0];
		specular_shading[1] = specularRef[1] * cosalpha * irradiance[1];
		specular_shading[2] = specularRef[2] * cosalpha * irradiance[2];

		color[0] = std::min(color[0] + diffuse_shading[0] + specular_shading[0] + mirrorRef[0]*reflection_shading[0], 255.f);
		color[1] = std::min(color[1] + diffuse_shading[1] + specular_shading[1] + mirrorRef[1]*reflection_shading[1], 255.f);
		color[2] = std::min(color[2] + diffuse_shading[2] + specular_shading[2] + mirrorRef[2]*reflection_shading[2], 255.f);
	}
	return color;
}

/*
 * Must render the scene from each camera's viewpoint and create an image.
 * You can use the methods of the Image class to save the image as a PPM file.
 */
void Scene::renderScene(void)
{

	size_t objectsSize = objects.size();
	size_t camerasSize = cameras.size();

	for(int i = 0; i < camerasSize; i++){
		Camera *cam = cameras[i];
		int nx = cam->imgPlane.nx;
		int ny = cam->imgPlane.ny;
		Image* image = new Image(nx, ny);

		for(int x = 0; x < nx; x++){
			for(int y = 0; y < ny; y++){

				Vector3f shading_vector;
				float tmin = std::numeric_limits<float>::max();
				Shape* shape = NULL;
				ReturnVal intersect_result, tempintersect_result;
				Color color;
				Ray ray = cam->getPrimaryRay(x, y);
				intersect_result.isIntersected = false;

				for(int k = 0; k <objectsSize; k++){
					Shape *obj = objects[k];
					tempintersect_result = obj->intersect(ray);

					if(tempintersect_result.isIntersected){
						float tempt = tempintersect_result.t;
						if(tempt < tmin+intTestEps && tempt > 0){
							tmin = tempt;
							shape = obj;
							intersect_result = tempintersect_result;
						}
					}
				}

				if(intersect_result.isIntersected){
					Vector3f wo = (-1 * ray.direction).normalized();
					shading_vector = shading(&ray, intersect_result, shape, wo, 0);

					color.red = shading_vector[0];
					color.grn = shading_vector[1];
					color.blu = shading_vector[2];
				}

				else {
					color.red = backgroundColor[0];
					color.grn = backgroundColor[1];
					color.blu = backgroundColor[2];
				}

				image->setPixelValue(x, y, color);
			}
		}

		image->saveImage(cam->imageName);
	}
}

// Parses XML file.
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLError eResult;
	XMLElement *pElement;

	maxRecursionDepth = 1;
	shadowRayEps = 0.001;

	eResult = xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	pElement = pRoot->FirstChildElement("MaxRecursionDepth");
	if(pElement != nullptr)
		pElement->QueryIntText(&maxRecursionDepth);

	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%f %f %f", &backgroundColor[0], &backgroundColor[1], &backgroundColor[2]);

	pElement = pRoot->FirstChildElement("ShadowRayEpsilon");
	if(pElement != nullptr)
		pElement->QueryFloatText(&shadowRayEps);

	pElement = pRoot->FirstChildElement("IntersectionTestEpsilon");
	if(pElement != nullptr)
		eResult = pElement->QueryFloatText(&intTestEps);

	// Parse cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while(pCamera != nullptr)
	{
        int id;
        char imageName[64];
        Vector3f pos, gaze, up;
        ImagePlane imgPlane;

		eResult = pCamera->QueryIntAttribute("id", &id);
		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &pos[0], &pos[1], &pos[2]);
		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &gaze[0], &gaze[1], &gaze[2]);
		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%f %f %f", &up[0], &up[1], &up[2]);
		camElement = pCamera->FirstChildElement("NearPlane");
		str = camElement->GetText();
		sscanf(str, "%f %f %f %f", &imgPlane.left, &imgPlane.right, &imgPlane.bottom, &imgPlane.top);
		camElement = pCamera->FirstChildElement("NearDistance");
		eResult = camElement->QueryFloatText(&imgPlane.distance);
		camElement = pCamera->FirstChildElement("ImageResolution");
		str = camElement->GetText();
		sscanf(str, "%d %d", &imgPlane.nx, &imgPlane.ny);
		camElement = pCamera->FirstChildElement("ImageName");
		str = camElement->GetText();
		strcpy(imageName, str);

		cameras.push_back(new Camera(id, imageName, pos, gaze, up, imgPlane));

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// Parse materals
	pElement = pRoot->FirstChildElement("Materials");
	XMLElement *pMaterial = pElement->FirstChildElement("Material");
	XMLElement *materialElement;
	while(pMaterial != nullptr)
	{
		materials.push_back(new Material());

		int curr = materials.size() - 1;

		eResult = pMaterial->QueryIntAttribute("id", &materials[curr]->id);
		materialElement = pMaterial->FirstChildElement("AmbientReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->ambientRef[0], &materials[curr]->ambientRef[1], &materials[curr]->ambientRef[2]);
		materialElement = pMaterial->FirstChildElement("DiffuseReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->diffuseRef[0], &materials[curr]->diffuseRef[1], &materials[curr]->diffuseRef[2]);
		materialElement = pMaterial->FirstChildElement("SpecularReflectance");
		str = materialElement->GetText();
		sscanf(str, "%f %f %f", &materials[curr]->specularRef[0], &materials[curr]->specularRef[1], &materials[curr]->specularRef[2]);
		materialElement = pMaterial->FirstChildElement("MirrorReflectance");
		if(materialElement != nullptr)
		{
			str = materialElement->GetText();
			sscanf(str, "%f %f %f", &materials[curr]->mirrorRef[0], &materials[curr]->mirrorRef[1], &materials[curr]->mirrorRef[2]);
		}
				else
		{
			materials[curr]->mirrorRef[0] = 0.0;
			materials[curr]->mirrorRef[1] = 0.0;
			materials[curr]->mirrorRef[2] = 0.0;
		}
		materialElement = pMaterial->FirstChildElement("PhongExponent");
		if(materialElement != nullptr)
			materialElement->QueryIntText(&materials[curr]->phongExp);

		pMaterial = pMaterial->NextSiblingElement("Material");
	}

	// Parse vertex data
	pElement = pRoot->FirstChildElement("VertexData");
	int cursor = 0;
	Vector3f tmpPoint;
	str = pElement->GetText();
	while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
		cursor++;
	while(str[cursor] != '\0')
	{
		for(int cnt = 0 ; cnt < 3 ; cnt++)
		{
			if(cnt == 0)
				tmpPoint[0] = atof(str + cursor);
			else if(cnt == 1)
				tmpPoint[1] = atof(str + cursor);
			else
				tmpPoint[2] = atof(str + cursor);
			while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
				cursor++;
			while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
				cursor++;
		}
		vertices.push_back(tmpPoint);
	}

	// Parse objects
	pElement = pRoot->FirstChildElement("Objects");

	// Parse spheres
	XMLElement *pObject = pElement->FirstChildElement("Sphere");
	XMLElement *objElement;
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int cIndex;
		float R;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Center");
		eResult = objElement->QueryIntText(&cIndex);
		objElement = pObject->FirstChildElement("Radius");
		eResult = objElement->QueryFloatText(&R);

		objects.push_back(new Sphere(id, matIndex, cIndex, R));

		pObject = pObject->NextSiblingElement("Sphere");
	}

	// Parse triangles
	pObject = pElement->FirstChildElement("Triangle");
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int p1Index;
		int p2Index;
		int p3Index;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Indices");
		str = objElement->GetText();
		sscanf(str, "%d %d %d", &p1Index, &p2Index, &p3Index);

		objects.push_back(new Triangle(id, matIndex, p1Index, p2Index, p3Index));

		pObject = pObject->NextSiblingElement("Triangle");
	}

	// Parse meshes
	pObject = pElement->FirstChildElement("Mesh");
	while(pObject != nullptr)
	{
		int id;
		int matIndex;
		int p1Index;
		int p2Index;
		int p3Index;
		int cursor = 0;
		int vertexOffset = 0;
		vector<Triangle> faces;

		eResult = pObject->QueryIntAttribute("id", &id);
		objElement = pObject->FirstChildElement("Material");
		eResult = objElement->QueryIntText(&matIndex);
		objElement = pObject->FirstChildElement("Faces");
		objElement->QueryIntAttribute("vertexOffset", &vertexOffset);
		str = objElement->GetText();
		while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
			cursor++;
		while(str[cursor] != '\0')
		{
			for(int cnt = 0 ; cnt < 3 ; cnt++)
			{
				if(cnt == 0)
					p1Index = atoi(str + cursor) + vertexOffset;
				else if(cnt == 1)
					p2Index = atoi(str + cursor) + vertexOffset;
				else
					p3Index = atoi(str + cursor) + vertexOffset;
				while(str[cursor] != ' ' && str[cursor] != '\t' && str[cursor] != '\n')
					cursor++;
				while(str[cursor] == ' ' || str[cursor] == '\t' || str[cursor] == '\n')
					cursor++;
			}
			faces.push_back(*(new Triangle(-1, matIndex, p1Index, p2Index, p3Index)));
		}

		objects.push_back(new Mesh(id, matIndex, faces));

		pObject = pObject->NextSiblingElement("Mesh");
	}

	// Parse lights
	int id;
	Vector3f position;
	Vector3f intensity;
	pElement = pRoot->FirstChildElement("Lights");

	XMLElement *pLight = pElement->FirstChildElement("AmbientLight");
	XMLElement *lightElement;
	str = pLight->GetText();
	sscanf(str, "%f %f %f", &ambientLight[0], &ambientLight[1], &ambientLight[2]);

	pLight = pElement->FirstChildElement("PointLight");
	while(pLight != nullptr)
	{
		eResult = pLight->QueryIntAttribute("id", &id);
		lightElement = pLight->FirstChildElement("Position");
		str = lightElement->GetText();
		sscanf(str, "%f %f %f", &position[0], &position[1], &position[2]);
		lightElement = pLight->FirstChildElement("Intensity");
		str = lightElement->GetText();
		sscanf(str, "%f %f %f", &intensity[0], &intensity[1], &intensity[2]);

		lights.push_back(new PointLight(position, intensity));

		pLight = pLight->NextSiblingElement("PointLight");
	}
}
