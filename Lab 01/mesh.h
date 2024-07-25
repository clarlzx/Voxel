#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <array>

// maximum number of vertices and triangles
#define MAXV 10000000
#define MAXT 10000000

typedef int OrTri;
typedef int tIdx;

inline OrTri makeOrTri(tIdx t, int version) { return (t << 3) | version; };
inline tIdx idx(OrTri ot) { return ot >> 3; };
inline int ver(OrTri ot) { return ot & 0b111; };
inline OrTri enext(OrTri ot) { return ot % 3 == 2 ? ot - 2 : ot + 1; };
inline OrTri sym(OrTri ot) { return ot - makeOrTri(idx(ot), 0) >= 3 ? ot - 3 : ot + 3; };

namespace {
	struct VectorHash {
		std::size_t operator()(const std::vector<double>& v) const {
			std::size_t seed = v.size();
			for (auto& d : v) {
				seed ^= std::hash<double>()(d) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
			return seed;
		}
	};
}

class myObjType {
	int vcount = 0;
	int tcount = 0;
	int voxelDepth = 4;
	int lastVoxelDepth = 0;
	bool isReadSTL = false;

	double vlist[MAXV][3];                                     // vertices list
	int tlist[MAXT][3];										   // triangle list
	double minMaxVlist[MAXT][6];							   // list of min and max x, y, z for each triangle
	int fnlist[MAXT][3];							           // fnext list for future
	double nlist[MAXT][3];									   // storing triangle normals
	double vnlist[MAXV][3];									   // storing vertex normals
	std::unordered_map<int, int> vCountMap;					   // key: idx of vertex, value: number of surrounding triangles
	std::unordered_map<std::string, std::vector<int>> edgeMap; // key: edge as a string "v0Idx, v1Idx"
															   // value: vector of OrTri of faces that share the same edge
	std::unordered_map<int, std::vector<std::vector<double>>> octTree; // key: depth of OctTree
																	   // value: min and max x, y, z of each subregion at a particular depth
	std::unordered_map<std::vector<double>, std::vector<int>, VectorHash> lastTIdxMap; // store overlapping triangle faces for 
																					   // max voxel depth in last computeOctTree call
																					   // key: subregion dimension, value: tIdx
																							

	double lmax[3]; // the maximum coordinates of x,y,z
	double lmin[3]; // the minimum coordinates of x,y,z

	int statMinAngle[18]; // each bucket is  degrees has a 10 degree range from 0 to 180 degree
	int statMaxAngle[18]; 


public:
	myObjType() { vcount = 0; tcount = 0; };
	void checkFileType(int fileType);
	void readFile(char* filename);  // assumming file contains a manifold
	void readObj(char* filename);
	void readStl(char* filename);
	void writeFile(char* filename);  
	void draw(bool m_Smooth);
	void drawVoxels();
	void drawBoundaries();
    void computeStat();
	void computeComponents();
	std::vector<int> getVertices(OrTri ot);
	int org(OrTri ot); 
	int dest(OrTri ot);
	void insertToEdgeMap(int tIdx);
	OrTri fnext(OrTri ot);
	void setupFnlist();

	// For voxelization
	void setVoxelDepth(int newVoxelDepth);
	void computeOctTree(int maxDepth);
	void setupMinMaxVList();
	std::array<std::vector<double>, 8> computeSubregion(const std::vector<double>& dim);
	std::vector<int> intersectCube(std::vector<double>& dim, std::vector<int>& prevOverlapTIdx);
};


