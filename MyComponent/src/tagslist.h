#ifndef TAGSLIST_H
#define TAGSLIST_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <math.h>


class ListaMarcas
{    
      
public:
   ListaMarcas(InnerModel *inner_);
   ~ListaMarcas();
  
  typedef struct
  {
    int id;
    float tx;
    float ty;
    float tz;
    float rx;
    float ry;
    float rz;
    QTime reloj;
  } Marca;
  
   void add(const RoboCompAprilTags::tag &t);
   Marca get(int id);
   bool exists(int id);
   float distance(int id);
   void borraMarca(int id);
   int getInitMark();
   void setInitMark(int id);
   bool getInMemory();
   void setInMemory(bool in);
   
   
   
      
private:
    Marca marca;
    QMap<int,Marca> lista;
    QMutex mutex;
    QVec memory;
    InnerModel *inner;
    bool inMemory;
    int initMark;
 
};

#endif

