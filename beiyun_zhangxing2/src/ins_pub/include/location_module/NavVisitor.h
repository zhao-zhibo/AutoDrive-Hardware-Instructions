#ifndef NAVVISITOR_H
#define NAVVISITOR_H



namespace navicore {

    class  IEpochVisit
    {
    public:
        IEpochVisit(){}
        virtual ~IEpochVisit(){}
        enum VisitType{START,EPOCH,FINISH,INFO};
    public:
        virtual bool EpochVisit(VisitType type,int epoch,char*strInfo) = 0;
    };

    //初始化进度
    class IProgressVisitor
    {
    public:
        virtual bool progressVisit(int value,char*strInfo) = 0;
    };

}

#endif 
