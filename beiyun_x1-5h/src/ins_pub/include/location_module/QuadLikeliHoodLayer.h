#pragma once
#include "navislam_global.h"
#include "Layer.h"
#include <fstream>
#include "QuadNode.h"
#include<math.h>

namespace navicore{
//转换后存储Likelihood Map Data 数据块定义
typedef struct tagQUADLLHMAP_HEAD
{
  int        m_iVersion;     //数据版本
  //数据范围
 double      m_dxy;          //地图分辨率
 double      m_dCenterX;     //地图中心
 double      m_dCenterY;     //地图中心
  int        m_iLevel;       //四叉树深度
} QUADLLHMAP_HEAD;

typedef struct tagQUADNODE_INFO
{
 QUADINX    m_iIndex;          //节点编号
 long       m_lOffset;         //数据库偏移量
 int        m_iDataFileIndex;  //数据文件索引
} QUADNODE_INFO;


class NAVISLAMSHARED_EXPORT QuadLikeliHoodLayer:
	public Layer
{
public:
	QuadLikeliHoodLayer();
	QuadLikeliHoodLayer(double dxy,double cx,double cy,int mapdepth);
	~QuadLikeliHoodLayer(void);
    void initMap(double dxy, double cx,double cy,int mapdepth);
    void reCenter(double cx,double cy);
    void copyfrom(QuadLikeliHoodLayer* pQuadLikeliHoodLayer); //copy the map layer
	//设置和读取模糊度值
	void  getLLHGradient(int level, double x, double y, double* GdtXYValue); //获取该点模糊度梯度值
	float getLLHValue(int level, double x, double y);
	bool  isEmpty();
	float getLLHValue(int level, double* pScanLineX,double* pScanLineY, int scanlen);
	void  setLLHValue(double x,  double y);
//	void  setLLHValue(double x,  double y,float value);

	void meanfilter(int n);
    double mapResolution(int level){return m_dxy*pow(2, m_iLevel - level);}
    int    maxlevel(){return m_iLevel; }
    bool updateMap(XY_ScanLine& xyScanLine); //更新地图
    bool updateMap(double cx,double cy,XY_ScanLine& xyScanLine);//Update Map with inner clear
	bool clearMap();               //清除数据内容
	bool load(const char* strName, const char* filePath); //文件加载地图
	bool save(const char* filePath);                      //保存文件
    QuadNode* rootNode(){return m_pRootNode; }
	Bound bound();   //计算数据范围
    QuadNode* queryQuadNode(int level, double x, double y); //查询四叉树节点
	double  m_dCenterX;      //地图中心点
    double  m_dCenterY;
    int     m_iLevel;  //四叉树深度
    double  m_dxy;     //图层最大分辨率
protected:
	QUADINX makeQuadIndex(QUADINX parentIndex,int posChild);
	QuadNode* getQuadNode(QUADINX id);
	void   initQuadNode(QuadNode* pNode); //初始化四叉树
    void   traverseReCenter(QuadNode* pNode);
    void   traverseCopyfrom(QuadNode* pNode,QuadNode* pfromNode);
	QuadNode* traverseQuadNode(QuadNode* parent, int level, double x,double y);  //遍历查询节点
	void traverseSave(QuadNode* pNode);  //遍历树保存
	void traverseSaveAs(void* pParam,QuadNode* pNode);
	void traverseBound(QuadNode* pNode); //计算有效地图范围
	void traverseClear(QuadNode* pNode); 
	void traverseMeanfilter(QuadNode* pNode,int n); 
	void traverseRemoveNoise(QuadNode* pNode,int minCount); 
	void traverseClearArea(QuadNode* pNode,double x,double y,double width,double height); 
	void traverseClearArea(QuadNode* pNode,double* x,double* y,int nPoints);
	QuadNode* m_pRootNode;   //四叉树根节点
	//保存数据文件
    std::ofstream* m_pIdxStream;
	std::ofstream* m_pMdtStream;
	int  m_iMdtFileCount;
	int  m_iQuadNodeCount;
	long m_iOffset;
	std::string    m_sMapPath;  //地图数据路径
	char m_bScanFlag[SCAN_SIZE];
};

}
