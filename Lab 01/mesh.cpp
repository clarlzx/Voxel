#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

#include "math.h"
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "mesh.h"
#include <queue>
#include <iomanip>
#include <numeric>

using namespace std;

const float DEFAULT_TMIN = 10.0e-4;
const float DEFAULT_TMAX = 10.0e6;

namespace {
	double magnitude(double vec[3]) {
		return sqrt(pow(vec[0], 2.0) + pow(vec[1], 2.0) + pow(vec[2], 2.0));
	}

	double dotProduct(double vec1[3], double vec2[3]) {
		return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
	}

	void crossProduct(double vec1[3], double vec2[3], double out[3]) {
		out[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
		out[1] = -(vec1[0] * vec2[2] - vec1[2] * vec2[0]);
		out[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
	}

	string to_3dp(string valStr){
		double val = atof(valStr.c_str());
		int temp = (int)(val * 1000 + 0.5); // to round up
		return to_string(temp / 1000.0);
	}

}

vector<int> myObjType::getVertices(OrTri ot) {
	int* vertices = tlist[idx(ot)];
	vector<int> result;
	int version = ver(ot);
	
	switch (version) {
	case 0:
		result.insert(result.end(), { vertices[0], vertices[1], vertices[2]});
		break;
	case 1: 
		result.insert(result.end(), { vertices[1], vertices[2], vertices[0] });
		break;
	case 2:
		result.insert(result.end(), { vertices[2], vertices[0], vertices[1] });
		break;
	case 3:
		result.insert(result.end(), { vertices[1], vertices[0], vertices[2] });
		break;
	case 4:
		result.insert(result.end(), { vertices[2], vertices[1], vertices[0] });
		break;
	case 5:
		result.insert(result.end(), { vertices[0], vertices[2], vertices[1] });
		break;
	default: 
		cout << "Unknown version" << endl;
	}
	return result;
}

int myObjType::org(OrTri ot) {
	return getVertices(ot)[0];
}

int myObjType::dest(OrTri ot) {
	return getVertices(ot)[1];
}

OrTri myObjType::fnext(OrTri ot) {
	if (ver(ot) > 2) {
		return sym(fnlist[idx(ot)][ver(ot) - 3]);
	}
	else {
		return fnlist[idx(ot)][ver(ot)];
	}
}

void myObjType::draw(bool m_Smooth) {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);

	for (int i = 1; i <= tcount; i++)
	{
		glBegin(GL_POLYGON);
		if (m_Smooth) {
			// Smooth shading on
			for (int j = 0; j < 3; j++) {
				glNormal3dv(vnlist[tlist[i][j]]);
				glVertex3dv(vlist[tlist[i][j]]);
			}
		}
		else {
			// Smooth shading off
			glNormal3dv(nlist[i]);
			for (int j = 0; j < 3; j++)
				glVertex3dv(vlist[tlist[i][j]]);
		}
		glEnd();
	}

	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void myObjType::drawVoxels() {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);

	glBegin(GL_QUADS);
	for (auto dim : octTree[voxelDepth]) {
		// front
		glNormal3d(0.0, 0.0, 1.0);
		glVertex3d(dim[3], dim[4], dim[5]);
		glVertex3d(dim[0], dim[4], dim[5]);
		glVertex3d(dim[0], dim[1], dim[5]);
		glVertex3d(dim[3], dim[1], dim[5]);

		// back
		glNormal3d(0.0, 0.0, -1.0);
		glVertex3d(dim[3], dim[1], dim[2]);
		glVertex3d(dim[0], dim[1], dim[2]);
		glVertex3d(dim[0], dim[4], dim[2]);
		glVertex3d(dim[3], dim[4], dim[2]);

		// right
		glNormal3d(1.0, 0.0, 0.0);
		glVertex3d(dim[3], dim[4], dim[2]);
		glVertex3d(dim[3], dim[4], dim[5]);
		glVertex3d(dim[3], dim[1], dim[5]);
		glVertex3d(dim[3], dim[1], dim[2]);

		// left
		glNormal3d(-1.0, 0.0, 0.0);
		glVertex3d(dim[0], dim[4], dim[5]);
		glVertex3d(dim[0], dim[4], dim[2]);
		glVertex3d(dim[0], dim[1], dim[2]);
		glVertex3d(dim[0], dim[1], dim[5]);

		// top
		glNormal3d(0.0, 1.0, 0.0);
		glVertex3d(dim[3], dim[4], dim[2]);
		glVertex3d(dim[0], dim[4], dim[2]);
		glVertex3d(dim[0], dim[4], dim[5]);
		glVertex3d(dim[3], dim[4], dim[5]);

		// bottom
		glNormal3d(0.0, -1.0, 0.0);
		glVertex3d(dim[3], dim[1], dim[5]);
		glVertex3d(dim[0], dim[1], dim[5]);
		glVertex3d(dim[0], dim[1], dim[2]);
		glVertex3d(dim[3], dim[1], dim[2]);
	}

	glEnd();

	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void myObjType::drawBoundaries() {

	glEnable(GL_LIGHTING);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glPushMatrix();
	double longestSide = 0.0;
	for (int i = 0; i < 3; i++)
		if ((lmax[i] - lmin[i]) > longestSide)
			longestSide = (lmax[i] - lmin[i]);
	glScalef(4.0 / longestSide, 4.0 / longestSide, 4.0 / longestSide);
	glTranslated(-(lmin[0] + lmax[0]) / 2.0, -(lmin[1] + lmax[1]) / 2.0, -(lmin[2] + lmax[2]) / 2.0);

	// For drawing boundary lines
	for (auto element = edgeMap.begin(); element != edgeMap.end(); element++) {
		// Access edgeMap items/element (first: key, second: value) 
		if (element->second.size() == 1) {
			glBegin(GL_LINES);
			int v0 = ver(element->second[0]) > 2 ? ver(element->second[0]) - 3 : ver(element->second[0]);
			int v1 = ver(element->second[0]) > 2 ? (ver(element->second[0]) - 1) % 3 : (ver(element->second[0]) + 1) % 3;
			glVertex3dv(vlist[tlist[idx(element->second[0])][v0]]);
			glVertex3dv(vlist[tlist[idx(element->second[0])][v1]]);
			glEnd();
		}
	}

	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void myObjType::writeFile(char* filename)
{
	string filenameStr(filename);
	string fileType = ".obj";
	if (fileType.compare(0, 4, filename, filenameStr.size() - 4, filenameStr.size()) != 0) {
		cout << "Wrong file type" << endl;
	}
	else {
		cout << "Writing " << filename << endl;
		ofstream outFile;
		outFile.open(filename);
		for (int v = 1; v <= vcount; v++) {
			outFile << "v " << vlist[v][0] << " " << vlist[v][1] << " " << vlist[v][2] << "\n";
		}
		for (int t = 1; t <= tcount; t++) {
			outFile << "f " << tlist[t][0] << " " << tlist[t][1] << " " << tlist[t][2] << "\n";
		}
		outFile.close();
		cout << "Writing done" << endl;
	}
}

void myObjType::checkFileType(int fileType) {
	if (fileType != 1 && fileType != 2) {
		cout << "Wrong number entered";
		exit(1);
	}
	if (fileType == 2) {
		isReadSTL = true;
	}
}

void myObjType::readFile(char* filename)
{
	string filenameStr(filename);
	string fileType;

	if (isReadSTL) {
		// STL
		fileType = ".stl";
		if (fileType.compare(0, 4, filename, filenameStr.size() - 4, filenameStr.size()) != 0) {
			cout << "Wrong file type" << endl;
			exit(1);
		}
		readStl(filename);
	}
	else {
		// OBJ
		string filenameStr(filename);
		fileType = ".obj";
		if (fileType.compare(0, 4, filename, filenameStr.size() - 4, filenameStr.size()) != 0) {
			cout << "Wrong file type" << endl;
			exit(1);
		}
		readObj(filename);
	}

	// Populate fnlist
	setupFnlist();

	// Populate minMaxVList
	setupMinMaxVList();

	cout << "No. of vertices: " << vcount << endl;
	cout << "No. of triangles: " << tcount << endl;
	computeStat();
	computeComponents();

	computeOctTree(voxelDepth);
}

// Edge map is used to set up fnlist easier.
// In this function, edges of triangle with index tIdx are added to edge map.
void myObjType::insertToEdgeMap(int tIdx) {
	const int* vIdx = tlist[tIdx];
	for (int k = 0; k < 3; k++) {
		string edge0 = to_string(vIdx[k]) + "," + to_string(vIdx[(k + 1) % 3]);
		string edge1 = to_string(vIdx[(k + 1) % 3]) + "," + to_string(vIdx[k]);
	
		if (edgeMap.find(edge0) != edgeMap.end()) {
			edgeMap[edge0].emplace_back(makeOrTri(tIdx, k));
		}
		else if (edgeMap.find(edge1) != edgeMap.end()) {
			edgeMap[edge1].emplace_back(makeOrTri(tIdx, k + 3));
		}
		else {
			vector<int> accumVector{ makeOrTri(tIdx, k) };
			edgeMap.insert({ edge0, accumVector });
		}
	}
}

void myObjType::setupFnlist() {
	for (int t = 1; t <= tcount; t++) {
		for (int ver = 0; ver < 3; ver++) {
			OrTri ot = makeOrTri(t, ver);
			vector<int>& vertices = getVertices(ot);
			string edge0 = to_string(vertices[0]) + "," + to_string(vertices[1]);
			string edge1 = to_string(vertices[1]) + "," + to_string(vertices[0]);

			// Assume a manifold, so one edge will have maximum 2 faces
			if (edgeMap.find(edge0) != edgeMap.end() && edgeMap.at(edge0).size() > 1) {
				fnlist[t][ver] = edgeMap.at(edge0)[0] == ot ? edgeMap.at(edge0)[1] : edgeMap.at(edge0)[0];
			}
			else if (edgeMap.find(edge1) != edgeMap.end() && edgeMap.at(edge1).size() > 1) {
				fnlist[t][ver] = edgeMap.at(edge1)[0] == sym(ot) ? sym(edgeMap.at(edge1)[1]) : sym(edgeMap.at(edge1)[0]);
			}
		}
	}
}

void myObjType::readObj(char* filename) 
{
	cout << "Opening " << filename << endl;
	ifstream inFile;
	inFile.open(filename);
	if (!inFile.is_open()) {
		cout << "We cannot find your file " << filename << endl;
		exit(1);
	}

	string line;
	int i, j;
	bool firstVertex = 1;
	double currCood;

	while (getline(inFile, line))
	{
		if ((line[0] == 'v' || line[0] == 'f') && line[1] == ' ')
		{
			if (line[0] == 'v')
			{
				vcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ') j++;
					currCood = vlist[vcount][k] = atof(line.substr(i, j - i).c_str());
					if (firstVertex) 
						lmin[k] = lmax[k] = currCood;
					else {
						if (lmin[k] > currCood)
							lmin[k] = currCood;
						if (lmax[k] < currCood)
							lmax[k] = currCood;
					}
					i = j;
				}

				firstVertex = 0;
			}
			if (line[0] == 'f')
			{
				tcount++;
				i = 1;
				const char* linec = line.data();
				for (int k = 0; k < 3; k++) {
					while (linec[i] == ' ') i++;
					j = i;
					while (linec[j] != ' ' && linec[j] != '\\') j++;
					tlist[tcount][k] = atof(line.substr(i, j - i).c_str());

					// Populate vCountMap to keep track of number of surrounding triangles each vertex has.
					// Used to calculate vnlist.
					if (vCountMap.find(tlist[tcount][k]) == vCountMap.end()) {
						vCountMap.insert({ tlist[tcount][k], 1 });
					}
					else {
						vCountMap[tlist[tcount][k]]++;
					}

					i = j;
					while (linec[j] != ' ') j++;

				}

				// Insert edges of this triangle to edgeMap.
				insertToEdgeMap(tcount);
			}
		}
	}

	// We suggest you to compute the normals here
	// Assuming that the vertices are arranged in counter-clockwise in .obj
	for (int i = 1; i <= tcount; i++) {
		int v0Idx = tlist[i][0];
		int v1Idx = tlist[i][1];
		int v2Idx = tlist[i][2];
		
		// Vector for normal computation
		double vec1[3] = { vlist[v1Idx][0] - vlist[v0Idx][0], 
						   vlist[v1Idx][1] - vlist[v0Idx][1],
						   vlist[v1Idx][2] - vlist[v0Idx][2] };
		double vec2[3] = { vlist[v2Idx][0] - vlist[v0Idx][0],
						   vlist[v2Idx][1] - vlist[v0Idx][1],
						   vlist[v2Idx][2] - vlist[v0Idx][2] };
		
		// Cross product of vectors to get normal vector
		nlist[i][0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
		nlist[i][1] = -(vec1[0] * vec2[2] - vec1[2] * vec2[0]);
		nlist[i][2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];

		// Normalize normal vector
		double mag = magnitude(nlist[i]);
		nlist[i][0] = nlist[i][0] / mag;
		nlist[i][1] = nlist[i][1] / mag;
		nlist[i][2] = nlist[i][2] / mag;

		// Add to vnlist.
		vnlist[v0Idx][0] += nlist[i][0] / vCountMap[v0Idx];
		vnlist[v0Idx][1] += nlist[i][1] / vCountMap[v0Idx];
		vnlist[v0Idx][2] += nlist[i][2] / vCountMap[v0Idx];

		vnlist[v1Idx][0] += nlist[i][0] / vCountMap[v1Idx];
		vnlist[v1Idx][1] += nlist[i][1] / vCountMap[v1Idx];
		vnlist[v1Idx][2] += nlist[i][2] / vCountMap[v1Idx];

		vnlist[v2Idx][0] += nlist[i][0] / vCountMap[v2Idx];
		vnlist[v2Idx][1] += nlist[i][1] / vCountMap[v2Idx];
		vnlist[v2Idx][2] += nlist[i][2] / vCountMap[v2Idx];
	}
}

void myObjType::readStl(char* filename)
{
	cout << "Opening " << filename << endl;
	ifstream inFile;
	inFile.open(filename);
	if (!inFile.is_open()) {
		cout << "We cannot find your file " << filename << endl;
		exit(1);
	}

	string line;
	int i, j;
	int t = 0;
	bool firstVertex = 1;
	double currCood;
	unordered_map<string, int> vMap;  // vertices map (for fast check of duplicated vertices)
									  // key: "x;y;z", value: vlist index of vertex

	while (getline(inFile, line))
	{
		i = 0;
		while (line[i] == ' ') i++;
		if (line[i] == 'f')
		{
			// Store normal
			tcount++;
			i = i + 12;
			const char* linec = line.data();
			for (int k = 0; k < 3; k++) {
				while (linec[i] == ' ') i++;
				j = i;
				while (linec[j] != ' ') j++;
				nlist[tcount][k] = atof(line.substr(i, j - i).c_str());
				i = j;
			}
			t = 0;
		}

		i = 0;
		while (line[i] == ' ') i++;
		if (line[i] == 'v')
		{
			i = i + 6;
			const char* linec = line.data();
			string coordStr = "";
			double coord[3];
			for (int k = 0; k < 3; k++) { // k is 0,1,2 for x,y,z
				while (linec[i] == ' ') i++;
				j = i;
				while (linec[j] != ' ') j++;

				// STL do not have vertex indexes, so vertex are relisted for each face.
				// There is hence the problem of duplicate vertices.
				// Round coordinates to 3.d.p. to mitigate floating point errors when checking for duplicate vertices.
				coordStr = coordStr == "" ? to_3dp(line.substr(i, j - i)) : coordStr + "," + to_3dp(line.substr(i, j - i));
				coord[k] = atof(line.substr(i, j - i).c_str());
				i = j;
			}

			// Check if vertices already exist in vlist using vMap.
			if (vMap.find(coordStr) != vMap.end()) {
				tlist[tcount][t] = vMap.at(coordStr);
			}
			else {
				vcount++;
				vMap.insert({ coordStr, vcount });
				tlist[tcount][t] = vcount;
				for (int k = 0; k < 3; k++) {
					currCood = vlist[vcount][k] = coord[k];
					if (firstVertex)
						lmin[k] = lmax[k] = currCood;
					else {
						if (lmin[k] > currCood)
							lmin[k] = currCood;
						if (lmax[k] < currCood)
							lmax[k] = currCood;
					}
				}

				// Populate vCountMap to keep track of number of surrounding triangles each vertex has.
				// Used to calculate vnlist.
				if (vCountMap.find(tlist[tcount][t]) == vCountMap.end()) {
					vCountMap.insert({ tlist[tcount][t], 1 });
				}
				else {
					vCountMap[tlist[tcount][t]]++;
				}
			}

			firstVertex = 0;
			t++;

			if (t == 3) {
				// Add to vnlist.
				vnlist[tlist[tcount][0]][0] += nlist[tcount][0] / vCountMap[tlist[tcount][0]];
				vnlist[tlist[tcount][0]][1] += nlist[tcount][1] / vCountMap[tlist[tcount][0]];
				vnlist[tlist[tcount][0]][2] += nlist[tcount][2] / vCountMap[tlist[tcount][0]];

				vnlist[tlist[tcount][1]][0] += nlist[tcount][0] / vCountMap[tlist[tcount][1]];
				vnlist[tlist[tcount][1]][1] += nlist[tcount][1] / vCountMap[tlist[tcount][1]];
				vnlist[tlist[tcount][1]][2] += nlist[tcount][2] / vCountMap[tlist[tcount][1]];

				vnlist[tlist[tcount][2]][0] += nlist[tcount][0] / vCountMap[tlist[tcount][2]];
				vnlist[tlist[tcount][2]][1] += nlist[tcount][1] / vCountMap[tlist[tcount][2]];
				vnlist[tlist[tcount][2]][2] += nlist[tcount][2] / vCountMap[tlist[tcount][2]];

				// Insert edges of this triangle to edgeMap.
				insertToEdgeMap(tcount);
			}
		}

	}
}

void myObjType::computeStat()
{
	int i;
    double minAngle = 0;
    double maxAngle = 0;
	int minCount = 0;

	for (int i = 1; i <= tcount; i++) {
		int v0Idx = tlist[i][0];
		int v1Idx = tlist[i][1];
		int v2Idx = tlist[i][2];

		// Vector for normal computation
		double vec01[3]{ vlist[v1Idx][0] - vlist[v0Idx][0],
						 vlist[v1Idx][1] - vlist[v0Idx][1],
						 vlist[v1Idx][2] - vlist[v0Idx][2] };
		double vec02[3]{ vlist[v2Idx][0] - vlist[v0Idx][0],
						 vlist[v2Idx][1] - vlist[v0Idx][1],
						 vlist[v2Idx][2] - vlist[v0Idx][2] };

		double vec10[3]{ vlist[v0Idx][0] - vlist[v1Idx][0],
						 vlist[v0Idx][1] - vlist[v1Idx][1],
						 vlist[v0Idx][2] - vlist[v1Idx][2] };
		double vec12[3]{ vlist[v2Idx][0] - vlist[v1Idx][0],
						 vlist[v2Idx][1] - vlist[v1Idx][1],
						 vlist[v2Idx][2] - vlist[v1Idx][2] };

		double angle[3];
		double tMinAngle, tMaxAngle;

		if (magnitude(vec01) == 0 || magnitude(vec02) == 0 || magnitude(vec12) == 0) {
			// If magnitude denominator is 0, angle should be 0
			tMinAngle = 0; 
			tMaxAngle = 0;
		}
		else {
			angle[0] = acos(max(-1, min(1, dotProduct(vec01, vec02) / (magnitude(vec01) * magnitude(vec02))))) * (180 / M_PI);
			angle[1] = acos(max(-1, min(1, dotProduct(vec10, vec12) / (magnitude(vec10) * magnitude(vec12))))) * (180 / M_PI);
			angle[2] = 180 - angle[0] - angle[1];
			tMinAngle = *min_element(begin(angle), end(angle));
			tMaxAngle = *max_element(begin(angle), end(angle));
			string tMaxStr = to_string(tMaxAngle);
		}

		// Convert to string to get correct index, as dividing such as angle / 10 will give inaccurate indexes sometimes
		string tMinStr = to_string(tMinAngle);
		string tMaxStr = to_string(tMaxAngle);
		string tMinIdx = tMinStr.substr(0, tMinStr.find('.') - 1); 
		string tMaxIdx = tMaxStr.substr(0, tMaxStr.find('.') - 1);

		statMinAngle[tMinIdx.empty() ? 0 : stoi(tMinIdx)]++;
		statMaxAngle[tMaxIdx.empty() ? 0 : stoi(tMaxIdx)]++;

		// Update overall minAngle and maxAngle
		if (tMinAngle < minAngle) {
			minAngle = tMinAngle;
		} else if (minCount == 0) {
			minCount++;
			minAngle = tMinAngle;
		}
		if (tMaxAngle > maxAngle) {
			maxAngle = tMaxAngle;
		}
	}

	cout << "Min. angle = " << minAngle << endl;
	cout << "Max. angle = " << maxAngle << endl;

	cout << "Statistics for Maximum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMaxAngle[i] << " ";
	cout << endl;
	cout << "Statistics for Minimum Angles" << endl;
	for (i = 0; i < 18; i++)
		cout << statMinAngle[i] << " ";
	cout << endl;
}

void myObjType::computeComponents() {
	int numComponents = 0;
	queue<int> tQueue;
	set<int> visitedT;

	for (int i = 1; i <= tcount; i++) {
		if (visitedT.find(i) == visitedT.end()) {
			numComponents++;
			tQueue.push(i);

			visitedT.insert(tQueue.front());

			while (!tQueue.empty()) {
				for (int j = 0; j < 3; j++) {
					int nextOrTri = fnext(makeOrTri(tQueue.front(), j));
					if (nextOrTri != 0 && visitedT.find(idx(nextOrTri)) == visitedT.end()) {
						visitedT.insert(idx(nextOrTri));
						tQueue.push(idx(nextOrTri));
					}
				}
				tQueue.pop();
			}
		}
	}
	cout << "Number of components: " << numComponents << endl;
}

void myObjType::setVoxelDepth(int newVoxelDepth) {
	voxelDepth = newVoxelDepth;
	cout << "Computing voxel levels..." << endl;
	computeOctTree(newVoxelDepth);
	cout << "Voxels computed" << endl;
}

void myObjType::setupMinMaxVList() {
	for (int i = 1; i <= tcount; i++) {
		minMaxVlist[i][0] = vlist[tlist[i][0]][0]; //minX
		minMaxVlist[i][1] = vlist[tlist[i][0]][1]; //minY
		minMaxVlist[i][2] = vlist[tlist[i][0]][2]; //minZ
		minMaxVlist[i][3] = vlist[tlist[i][0]][0]; //maxX
		minMaxVlist[i][4] = vlist[tlist[i][0]][1]; //maxY
		minMaxVlist[i][5] = vlist[tlist[i][0]][2]; //maxZ

		for (int j = 1; j < 3; j++) {
			if (vlist[tlist[i][j]][0] < minMaxVlist[i][0]) {
				minMaxVlist[i][0] = vlist[tlist[i][j]][0];
			}
			if (vlist[tlist[i][j]][1] < minMaxVlist[i][1]) {
				minMaxVlist[i][1] = vlist[tlist[i][j]][1];
			}
			if (vlist[tlist[i][j]][2] < minMaxVlist[i][2]) {
				minMaxVlist[i][2] = vlist[tlist[i][j]][2];
			}
			if (vlist[tlist[i][j]][0] > minMaxVlist[i][3]) {
				minMaxVlist[i][3] = vlist[tlist[i][j]][0];
			}
			if (vlist[tlist[i][j]][1] > minMaxVlist[i][4]) {
				minMaxVlist[i][4] = vlist[tlist[i][j]][1];
			}
			if (vlist[tlist[i][j]][2] > minMaxVlist[i][5]) {
				minMaxVlist[i][5] = vlist[tlist[i][j]][2];
			}
		}
	}
}

// Checks if triangle intersects cube by Seperating Axis Theorem (SAT)
bool sat(const vector<double>& cubeDim, const double v0[3], const double v1[3], const double v2[3]) {
	double cubeCenter[3]{ (cubeDim[0] + cubeDim[3]) / 2,
						  (cubeDim[1] + cubeDim[4]) / 2,
						  (cubeDim[2] + cubeDim[5]) / 2 };
	double cubeExtents[3]{ abs(cubeDim[3] - cubeDim[0]),
						   abs(cubeDim[4] - cubeDim[1]),
						   abs(cubeDim[5] - cubeDim[2]) };

	// Translate cube and triangle to origin
	double tVert0[3]{ v0[0] - cubeCenter[0], v0[1] - cubeCenter[1], v0[2] - cubeCenter[2] };
	double tVert1[3]{ v1[0] - cubeCenter[0], v1[1] - cubeCenter[1], v1[2] - cubeCenter[2] };
	double tVert2[3]{ v2[0] - cubeCenter[0], v2[1] - cubeCenter[1], v2[2] - cubeCenter[2] };

	// Get triangle edges
	double edge0[3]{ v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2] };
	double edge1[3]{ v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] };
	double edge2[3]{ v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };

	// Possible separating axes
	double sepAxes[13][3]{};

	// 3 cube face normals after its center has been translated to the origin
	double xAxis[3]{ 1.0, 0.0, 0.0 };
	double yAxis[3]{ 0.0, 1.0, 0.0 };
	double zAxis[3]{ 0.0, 0.0, 1.0 };
	memcpy(sepAxes, xAxis, sizeof(xAxis));
	memcpy(sepAxes + 1, yAxis, sizeof(xAxis));
	memcpy(sepAxes + 2, zAxis, sizeof(xAxis));
	
	// 1 Triangle normal
	crossProduct(edge0, edge1, sepAxes[3]);

	// 9 cross products of 1 triangle edge and 1 cube face edge
	crossProduct(xAxis, edge0, sepAxes[4]);
	crossProduct(xAxis, edge1, sepAxes[5]);
	crossProduct(xAxis, edge2, sepAxes[6]);
	
	crossProduct(yAxis, edge0, sepAxes[7]);
	crossProduct(yAxis, edge1, sepAxes[8]);
	crossProduct(yAxis, edge2, sepAxes[9]);
	
	crossProduct(zAxis, edge0, sepAxes[10]);
	crossProduct(zAxis, edge1, sepAxes[11]);
	crossProduct(zAxis, edge2, sepAxes[12]);

	for (int i = 0; i < 13; ++i) {
		double p0 = dotProduct(tVert0, sepAxes[i]);
		double p1 = dotProduct(tVert1, sepAxes[i]);
		double p2 = dotProduct(tVert2, sepAxes[i]);

		double r = cubeExtents[0] * abs(dotProduct(xAxis, sepAxes[i])) +
				   cubeExtents[1] * abs(dotProduct(yAxis, sepAxes[i])) +
				   cubeExtents[2] * abs(dotProduct(zAxis, sepAxes[i]));

		if (max(-max(max(p0, p1), p2), min(min(p0, p1), p2)) > r) {
			return false;
		}
	}
	
	return true;
}

// Returns true if there are triangular faces that intersect the cube
vector<int> myObjType::intersectCube(vector<double>& cubeDim, vector<int>& tIndices) {
	vector<int> overlapTIdx;

	for (auto tIdx : tIndices) {

		if ((cubeDim[0] <= minMaxVlist[tIdx][3]) && (cubeDim[3] >= minMaxVlist[tIdx][0]) && // determine overlap in x plane
			(cubeDim[1] <= minMaxVlist[tIdx][4]) && (cubeDim[4] >= minMaxVlist[tIdx][1]) && // determine overlap in y plane
			(cubeDim[2] <= minMaxVlist[tIdx][5]) && (cubeDim[5] >= minMaxVlist[tIdx][2]))   // determine overlap in z plane
		{
			if (sat(cubeDim, vlist[tlist[tIdx][0]], vlist[tlist[tIdx][1]], vlist[tlist[tIdx][2]])) {
				overlapTIdx.emplace_back(tIdx);
			}
		}
	}

	return overlapTIdx;
}

array<vector<double>, 8> myObjType::computeSubregion(const vector<double>& dim) {
	
	array<vector<double>, 8> nextDim;
	
	double midX = dim[0] + (dim[3] - dim[0]) / 2;
	double midY = dim[1] + (dim[4] - dim[1]) / 2;
	double midZ = dim[2] + (dim[5] - dim[2]) / 2;

	// 1st cubic subregion
	nextDim[0] = vector<double>{ midX, dim[1], dim[2], dim[3], midY, midZ };

	// 2nd cubic subregion
	nextDim[1] = vector<double>{ dim[0], dim[1], dim[2], midX, midY, midZ };

	// 3rd cubic subregion
	nextDim[2] = vector<double>{ midX, midY, dim[2], dim[3], dim[4], midZ };

	// 4th cubic subregion
	nextDim[3] = vector<double>{ dim[0], midY, dim[2], midX, dim[4], midZ };
	
	// 5th cubic subregion
	nextDim[4] = vector<double>{ midX, dim[1], midZ, dim[3], midY, dim[5] };

	// 6th cubic subregion
	nextDim[5] = vector<double>{ dim[0], dim[1], midZ, midX, midY, dim[5] };
	
	// 7th cubic subregion
	nextDim[6] = vector<double>{ midX, midY, midZ, dim[3], dim[4], dim[5] };
	
	// 8th cubic subregion
	nextDim[7] = vector<double>{ dim[0], midY, midZ, midX, dim[4], dim[5] };

	return nextDim;
}

void myObjType::computeOctTree(int maxDepth) {

	vector<int> overlapTIdx;
	// Keeps track of triangles that intersect with region to avoid searching all triangles again, 
	// when checking for overlap in subsequent subregions.
	queue<vector<int>> intersectQueue;
	int depth = 0;

	if (maxDepth <= lastVoxelDepth) {
		// do nothing as already exists
		return;
	} else if (lastVoxelDepth == 0) {
		octTree.insert({ 0,  {{lmin[0],
							   lmin[1],
							   lmin[2],
							   lmax[0],
							   lmax[1],
							   lmax[2]}} });
		overlapTIdx.resize(tcount);
		iota(overlapTIdx.begin(), overlapTIdx.end(), 1);
	} else {
		for (auto it = lastTIdxMap.begin(); it != lastTIdxMap.end(); ++it) {
			array<vector<double>, 8> nextDim = computeSubregion(it->first);
			octTree.insert({ lastVoxelDepth + 1, {} });
			for (auto dim : nextDim) {
				octTree[lastVoxelDepth + 1].emplace_back(dim);
			}
			intersectQueue.push(it->second);
		}
		depth = lastVoxelDepth + 1;
		lastTIdxMap.clear();
	}

	lastVoxelDepth = maxDepth;
	
	while (depth <= maxDepth) {
		cout << "Computing OctTree depth " << depth << endl;

		int count = 0;

		// derive 8 subregions		
		auto it = octTree[depth].begin();
		while (it != octTree[depth].end()) {
			// *it: dimension of regions at current depth
			count++; 

			if (depth != 0) {
				overlapTIdx = intersectCube(*it, intersectQueue.front());
			}

			if (count % 8 == 0) {
				// All subregions of current depth have checked intersection with triangle
				// indices from previous depth, hence remove triangle indices from queue
				intersectQueue.pop();
				count = 0;
			}
			
			if (overlapTIdx.empty()) {
				// remove region from octTree
				it = octTree[depth].erase(it);
				continue;
			}
			else if (depth == maxDepth) {
				// store overlapTIdx for future computation if rendered voxel depth is changed
				lastTIdxMap.insert({ *it, overlapTIdx });
			}
			else if (depth < maxDepth) {
				// Searches at level after maxDepth, hence no need to do during last iteration
				array<vector<double>, 8> nextDim = computeSubregion(*it);
				
				if (octTree.find(depth + 1) == octTree.end()) {
					octTree.insert({ depth + 1, {} });
				}

				for (auto dim : nextDim) {
					octTree[depth + 1].emplace_back(dim);
				}

				intersectQueue.push(overlapTIdx);
			}	
			it++;
		}

		depth++;
	}

}