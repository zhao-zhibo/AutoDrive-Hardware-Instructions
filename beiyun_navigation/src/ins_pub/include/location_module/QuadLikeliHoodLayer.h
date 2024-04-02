#pragma once
#include "navislam_global.h"
#include "Layer.h"
#include <fstream>
#include "QuadNode.h"
#include<math.h>

namespace navicore{
//ת����洢Likelihood Map Data ���ݿ鶨��
typedef struct tagQUADLLHMAP_HEAD
{
  int        m_iVersion;     //���ݰ汾
  //���ݷ�Χ
 double      m_dxy;          //��ͼ�ֱ���
 double      m_dCenterX;     //��ͼ����
 double      m_dCenterY;     //��ͼ����
  int        m_iLevel;       //�Ĳ������
} QUADLLHMAP_HEAD;

typedef struct tagQUADNODE_INFO
{
 QUADINX    m_iIndex;          //�ڵ���
 long       m_lOffset;         //���ݿ�ƫ����
 int        m_iDataFileIndex;  //�����ļ�����
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
	//���úͶ�ȡģ����ֵ
	void  getLLHGradient(int level, double x, double y, double* GdtXYValue); //��ȡ�õ�ģ�����ݶ�ֵ
	float getLLHValue(int level, double x, double y);
	bool  isEmpty();
	float getLLHValue(int level, double* pScanLineX,double* pScanLineY, int scanlen);
	void  setLLHValue(double x,  double y);
//	void  setLLHValue(double x,  double y,float value);

	void meanfilter(int n);
    double mapResolution(int level){return m_dxy*pow(2, m_iLevel - level);}
    int    maxlevel(){return m_iLevel; }
    bool updateMap(XY_ScanLine& xyScanLine); //���µ�ͼ
    bool updateMap(double cx,double cy,XY_ScanLine& xyScanLine);//Update Map with inner clear
	bool clearMap();               //�����������
	bool load(const char* strName, const char* filePath); //�ļ����ص�ͼ
	bool save(const char* filePath);                      //�����ļ�
    QuadNode* rootNode(){return m_pRootNode; }
	Bound bound();   //�������ݷ�Χ
    QuadNode* queryQuadNode(int level, double x, double y); //��ѯ�Ĳ����ڵ�
	double  m_dCenterX;      //��ͼ���ĵ�
    double  m_dCenterY;
    int     m_iLevel;  //�Ĳ������
    double  m_dxy;     //ͼ�����ֱ���
protected:
	QUADINX makeQuadIndex(QUADINX parentIndex,int posChild);
	QuadNode* getQuadNode(QUADINX id);
	void   initQuadNode(QuadNode* pNode); //��ʼ���Ĳ���
    void   traverseReCenter(QuadNode* pNode);
    void   traverseCopyfrom(QuadNode* pNode,QuadNode* pfromNode);
	QuadNode* traverseQuadNode(QuadNode* parent, int level, double x,double y);  //������ѯ�ڵ�
	void traverseSave(QuadNode* pNode);  //����������
	void traverseSaveAs(void* pParam,QuadNode* pNode);
	void traverseBound(QuadNode* pNode); //������Ч��ͼ��Χ
	void traverseClear(QuadNode* pNode); 
	void traverseMeanfilter(QuadNode* pNode,int n); 
	void traverseRemoveNoise(QuadNode* pNode,int minCount); 
	void traverseClearArea(QuadNode* pNode,double x,double y,double width,double height); 
	void traverseClearArea(QuadNode* pNode,double* x,double* y,int nPoints);
	QuadNode* m_pRootNode;   //�Ĳ������ڵ�
	//���������ļ�
    std::ofstream* m_pIdxStream;
	std::ofstream* m_pMdtStream;
	int  m_iMdtFileCount;
	int  m_iQuadNodeCount;
	long m_iOffset;
	std::string    m_sMapPath;  //��ͼ����·��
	char m_bScanFlag[SCAN_SIZE];
};

}
