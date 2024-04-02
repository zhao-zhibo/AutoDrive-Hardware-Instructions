#pragma once
#include "navislam_global.h"
#include "Layer.h"
#include <stdio.h>
#include <fstream>
#include <unistd.h>


namespace navicore
{

#define NODE_XY_CELL_SIZE 1000     //�Ĳ���ƽ������ߴ�
#define NODE_Z_CELL_SIZE  50000    //Z�����������


/*
	NV_UINT64  Ox  FFFF      FFFF     FFFF        FFFF 
            Z-index    R      Intensity  Occupied-Value      	            
*/

typedef struct tagEXQUADLLHMAP_HEAD
{
	int        m_iVersion;      //���ݰ汾
	//���ݷ�Χ
	double      m_dxyz;          //��ͼ�ֱ���
	double      m_dCenterX;     //��ͼ����
	double      m_dCenterY;     //��ͼ����
	double      m_dCenterZ;     //��ͼ����
	int         m_iLevel;        //�Ĳ������
} EXQUADLLHMAP_HEAD;


typedef struct tagEXQUADNODE_INFO
{
	NV_UINT64  m_iIndex;          //�ڵ���
    int32_t     m_lOffset;         //���ݿ�ƫ����
	int        m_iDataFileIndex;  //�����ļ�����
} EXQUADNODE_INFO;



typedef struct tagNodeZGrid{
	public:
		bool  getOccupiedValue(NV_UINT16 zi, NV_UINT16& intValue, NV_UINT16& ocValue);
		void  setOccupiedValue(NV_UINT16 zi, NV_UINT16 intensity);
		std::vector<NV_UINT64> m_CellValues;
	private:
		bool  cellbinarySearch(NV_UINT64 val, std::vector<NV_UINT64>::iterator& it); //���ֲ���դ��λ��
}NodeZGrid;

//��չ�Ĳ����ڵ�
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
	double  m_dCenterX; //���ĵ�
	double  m_dCenterY;
	double  m_dCenterZ;
	NV_UINT64 m_iIndex;
	int       m_iLevel;
	double    m_dxy;
	double    m_dz;
	NodeZGrid*  m_pNodeZGridArray;  //Z������

	//�ӽڵ�
	ExQuadNode* m_pChild[4];  //00 01 10 11
	ExQuadNode* m_pParent;    //���ڵ�
};

//��չ�Ĳ���
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
	ExQuadNode* queryQuadNode(int level, double x, double y); //��ѯ�Ĳ����ڵ�
	ExQuadNode* rootNode(){ return m_pRootNode; }
	bool isEmpty();
	int maxLevel(){ return m_iLevel; }
	bool load(const char* filePath); //�ļ����ص�ͼ
	bool save(const char* filePath); //�����ļ�
	bool saveAs(const char* filePath);//��ͼ�������
protected:
	ExQuadNode*  traverseQuadNode(ExQuadNode* pNode, int level, double x, double y);
	void traverseSave(ExQuadNode* pNode);  //����������
	NV_UINT64 makeQuadIndex(NV_UINT64 parentIndex, int posChild);
	ExQuadNode* getQuadNode(NV_UINT64 id);
	void    initQuadNode(ExQuadNode* pNode); //��ʼ���Ĳ���
    int     m_iVersion;
    int     m_iLevel;   //�Ĳ������
	double  m_dxyz;     //ͼ�����ֱ���
	double  m_dCenterX; //��ͼ���ĵ�
	double  m_dCenterY;
	double  m_dCenterZ;
	ExQuadNode* m_pRootNode; //�Ĳ������ڵ�

	//���������ļ�
	std::ofstream* m_pIdxStream;
	std::ofstream* m_pMdtStream;
	int  m_iMdtFileCount;
	int  m_iQuadNodeCount;
	long m_iOffset;
	std::string  m_sMapPath;  //��ͼ����·��
};


}

