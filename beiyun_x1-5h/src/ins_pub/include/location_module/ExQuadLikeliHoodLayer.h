#pragma once
#include "navislam_global.h"
#include "Layer.h"
#include <stdio.h>
#include <fstream>
#include <unistd.h>


namespace navicore
{

#define NODE_XY_CELL_SIZE 1000     //四叉树平面格网尺寸
#define NODE_Z_CELL_SIZE  50000    //Z方向格网划分


/*
	NV_UINT64  Ox  FFFF      FFFF     FFFF        FFFF 
            Z-index    R      Intensity  Occupied-Value      	            
*/

typedef struct tagEXQUADLLHMAP_HEAD
{
	int        m_iVersion;      //数据版本
	//数据范围
	double      m_dxyz;          //地图分辨率
	double      m_dCenterX;     //地图中心
	double      m_dCenterY;     //地图中心
	double      m_dCenterZ;     //地图中心
	int         m_iLevel;        //四叉树深度
} EXQUADLLHMAP_HEAD;


typedef struct tagEXQUADNODE_INFO
{
	NV_UINT64  m_iIndex;          //节点编号
    int32_t     m_lOffset;         //数据库偏移量
	int        m_iDataFileIndex;  //数据文件索引
} EXQUADNODE_INFO;



typedef struct tagNodeZGrid{
	public:
		bool  getOccupiedValue(NV_UINT16 zi, NV_UINT16& intValue, NV_UINT16& ocValue);
		void  setOccupiedValue(NV_UINT16 zi, NV_UINT16 intensity);
		std::vector<NV_UINT64> m_CellValues;
	private:
		bool  cellbinarySearch(NV_UINT64 val, std::vector<NV_UINT64>::iterator& it); //二分查找栅格位置
}NodeZGrid;

//扩展四叉树节点
class NAVISLAMSHARED_EXPORT ExQuadNode
{
public:
	ExQuadNode(void);
	~ExQuadNode(void);

	bool contains(double x, double y){
		return  (x >= m_envNode.m_lfXO) &&
				(x <= m_envNode.m_lfXE) &&
				(y >= m_envNode.m_lfYO) &&
				(y <= m_envNode.m_lfYE);
	}
		
	float getOcValue(double x, double y, double z);
	NV_UINT16 getIntensity(double x, double y, double z);
	void getOcAndIntValue(double x, double y, double z, NV_UINT16& intensityValue, float& ocValue);
	void getOccupiedValue(double x, double y, double z, NV_UINT16& intensityValue, NV_UINT16& ocValue);
	void setOccupiedValue(double x, double y, double z, NV_UINT16 intValue);
	void getOcGradient(double x, double y, double z, double* gradient);
	Bound     m_envNode;
	double  m_dCenterX; //中心点
	double  m_dCenterY;
	double  m_dCenterZ;
	NV_UINT64 m_iIndex;
	int       m_iLevel;
	double    m_dxy;
	double    m_dz;
	NodeZGrid*  m_pNodeZGridArray;  //Z列数组

	//子节点
	ExQuadNode* m_pChild[4];  //00 01 10 11
	ExQuadNode* m_pParent;    //父节点
};

//扩展四叉树
class NAVISLAMSHARED_EXPORT ExQuadLikeliHoodLayer :
	public Layer
{
public:
	ExQuadLikeliHoodLayer();
	ExQuadLikeliHoodLayer(double dxyz, double cx, double cy, double cz, int mapdepth);
	~ExQuadLikeliHoodLayer();
	float getOcValue(int level, double x, double y, double z);
	NV_UINT16 getIntensity(int level, double x, double y, double z);
	void getOcGradient(int level,double x, double y, double z, double* gradient);
	void setOccupiedValue(double x, double y, double z, NV_UINT16 intValue = 0);
	ExQuadNode* queryQuadNode(int level, double x, double y); //查询四叉树节点
	ExQuadNode* rootNode(){ return m_pRootNode; }
	bool isEmpty();
	int maxLevel(){ return m_iLevel; }
	bool load(const char* filePath); //文件加载地图
	bool save(const char* filePath); //保存文件
	bool saveAs(const char* filePath);//地图数据另存
protected:
	ExQuadNode*  traverseQuadNode(ExQuadNode* pNode, int level, double x, double y);
	void traverseSave(ExQuadNode* pNode);  //遍历树保存
	NV_UINT64 makeQuadIndex(NV_UINT64 parentIndex, int posChild);
	ExQuadNode* getQuadNode(NV_UINT64 id);
	void    initQuadNode(ExQuadNode* pNode); //初始化四叉树
    int     m_iVersion;
    int     m_iLevel;   //四叉树深度
	double  m_dxyz;     //图层最大分辨率
	double  m_dCenterX; //地图中心点
	double  m_dCenterY;
	double  m_dCenterZ;
	ExQuadNode* m_pRootNode; //四叉树根节点

	//保存数据文件
	std::ofstream* m_pIdxStream;
	std::ofstream* m_pMdtStream;
	int  m_iMdtFileCount;
	int  m_iQuadNodeCount;
	long m_iOffset;
	std::string  m_sMapPath;  //地图数据路径
};


}

