#include "MyGL.h"
#include <glm/gtc/matrix_access.hpp>
//------------------------------------------------------------------------------
MyGL::MyGL()
    :
    _doTriangulate( false ),
    _doRasterize( false ) {
}

//------------------------------------------------------------------------------
MyGL::~MyGL()
{}

//------------------------------------------------------------------------------
bool MyGL::TriangulatePolygon( const vector<GLVertex> &polygonVerts,
                               vector<GLVertex> &triangleVerts ) {
    if( !_doTriangulate )
        return false;
    
	if (polygonVerts.size() >= 3) {
		int triNum = polygonVerts.size() - 2;
		for (int i = 1; i < triNum + 1; i++) {
			triangleVerts.push_back(polygonVerts[0]);
			triangleVerts.push_back(polygonVerts[i]);
			triangleVerts.push_back(polygonVerts[i + 1]);
		}
		return true;
	}
	else {
		return true;
	}
	
}

//------------------------------------------------------------------------------
bool MyGL::RasterizeTriangle(GLVertex verts[3]) {
	if (!_doRasterize)
		return false;

	GLVertex& v0 = verts[0];
	GLVertex& v1 = verts[1];
	GLVertex& v2 = verts[2];

	// get the edge equations
	glm::vec3 e01 = glm::vec3(v0.position[1] - v1.position[1], v1.position[0] - v0.position[0], v0.position[0] * v1.position[1] - v1.position[0] * v0.position[1]);
	glm::vec3 e12 = glm::vec3(v1.position[1] - v2.position[1], v2.position[0] - v1.position[0], v1.position[0] * v2.position[1] - v2.position[0] * v1.position[1]);
	glm::vec3 e20 = glm::vec3(v2.position[1] - v0.position[1], v0.position[0] - v2.position[0], v2.position[0] * v0.position[1] - v0.position[0] * v2.position[1]);

	// check the result of edge equation of the center of triangle 
	glm::vec3 center = glm::vec3((v0.position[0] + v1.position[0] + v2.position[0]) / 3, (v0.position[1] + v1.position[1] + v2.position[1]) / 3, 1);
	int val0 = dot(e01, center);
	int val1 = dot(e12, center);
	int val2 = dot(e20, center);
	bool edge = true;
	if ((val0 < 0) && (val1 < 0) && (val2 < 0)) edge = false;		// if edge is false, point clockwise, i will negate the result of edge equation
	
	// get color of each vertex
	glm::vec4 color0, color1, color2;
	for (int i = 0; i < 3; i++) {
		GLVertex &v = verts[i];
		glm::vec4 color;
		if (textureEnabled) {
			if (texture.id != 0) {
				// look up color in the texture
				int x = v.texCoord[0] * (texture.width - 1);
				int y = v.texCoord[1] * (texture.height - 1);
				color = texture.GetPixel(x, y, 0);
			}
		}
		else
			color = v.color;

		if (i == 0) color0 = color;
		else if (i == 1) color1 = color;
		else if (i == 2) color2 = color;
	}

	// get interpolation equation
	float A = (e01[2] + e12[2] + e20[2]);
	glm::mat3 Es; 
	Es[0] = e12; Es[1] = e20; Es[2] = e01;
	Es = transpose(Es);

	// cull if triangle is back-face
	if (cullFaceEnabled) {
		if (A <= 0) return false;
	}

	glm::vec3 linInterpR = (1 / A) * glm::vec3(color0[0], color1[0], color2[0]) * Es;
	glm::vec3 linInterpG = (1 / A) * glm::vec3(color0[1], color1[1], color2[1]) * Es;
	glm::vec3 linInterpB = (1 / A) * glm::vec3(color0[2], color1[2], color2[2]) * Es;
	glm::vec3 linInterpA = (1 / A) * glm::vec3(color0[3], color1[3], color2[3]) * Es;
	glm::vec3 linInterpZ = (1 / A) * glm::vec3(v0.position[2], v1.position[2], v2.position[2]) * Es;

	// find ymax and ymin
	int yindex = FindYMax(verts);
	int ymax = verts[yindex].position[1];
	int ymin;
	if (verts[(yindex + 1) % 3].position[1] > verts[(yindex + 2)% 3].position[1])
		ymin = verts[(yindex + 2) % 3].position[1];
	else
		ymin = verts[(yindex + 1) % 3].position[1];

	// find xmax and xmin
	int xindex = FindXMax(verts);
	int xmax = verts[xindex].position[0];
	int xmin;
	if (verts[(xindex + 1) % 3].position[0] > verts[(xindex + 2) % 3].position[0])
		xmin = verts[(xindex + 2) % 3].position[0];
	else
		xmin = verts[(xindex + 1) % 3].position[0];
    
	int w, h;
	frameBuffer.GetSize(w, h); 
	for (int y = ymax; y >= ymin; y--) {
		for (int x = xmin; x <= xmax; x++) {
			if (EdgeCheck(x, y, e01, e12, e20, edge)) {
				if (y < 0 || x < 0 || y >= h || x >= w) 
					continue;
				float depth = dot(linInterpZ, glm::vec3(x, y, 1.0));
				float rv = dot(linInterpR, glm::vec3(x, y, 1.0));
				float gv = dot(linInterpG, glm::vec3(x, y, 1.0));
				float bv = dot(linInterpB, glm::vec3(x, y, 1.0));
				if ((rv == gv) && (bv == 2*rv)) depth = 0.99;									// it is background, so change it to 0.99
				if (depth < 0) continue;
				if (depth < frameBuffer.GetDepth(x, y)) {
					frameBuffer.SetDepth(x, y, depth);
					InterpolateColor(x, y, linInterpR, linInterpG, linInterpB, linInterpA);
				}
			}
		}
	}

	return true;
}

bool MyGL::EdgeCheck(float x, float y, glm::vec3 e01, glm::vec3 e12, glm::vec3 e20, bool edge) {
	glm::vec3 pos = glm::vec3(x, y, 1.0);
	int val0 = dot(e01, pos);
	int val1 = dot(e12, pos);
	int val2 = dot(e20, pos);

	if (edge) {
		if ((val0 >= 0) && (val1 >= 0) && (val2 >= 0)) return true;
		else return false;
	}
	else {
		if ((val0 < 0) && (val1 < 0) && (val2 < 0)) return true;
		else return false;
	}
}

void MyGL::InterpolateColor(int x, int y, glm::vec3 linInterpR, glm::vec3 linInterpG, glm::vec3 linInterpB, glm::vec3 linInterpA) {
	float rv = dot(linInterpR, glm::vec3(x, y, 1.0));
	float gv = dot(linInterpG, glm::vec3(x, y, 1.0));
	float bv = dot(linInterpB, glm::vec3(x, y, 1.0));
	float av = dot(linInterpA, glm::vec3(x, y, 1.0));
	frameBuffer.SetPixel(x, y, glm::vec4(rv, gv, bv, av));
}

int MyGL::FindYMax(GLVertex verts[3]) {
	for (int i = 0; i < 3; i++) {
		if ((verts[i].position[1] >= verts[(i + 1) % 3].position[1]) && (verts[i].position[1] >= verts[(i + 2) % 3].position[1]))
			return i;
	}
}

int MyGL::FindXMax(GLVertex verts[3]) {
	for (int i = 0; i < 3; i++) {
		if ((verts[i].position[0] >= verts[(i + 1) % 3].position[0]) && (verts[i].position[0] >= verts[(i + 2) % 3].position[0]))
			return i;
	}
}