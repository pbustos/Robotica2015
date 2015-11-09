#include "tagslist.h"
 
ListaMarcas::ListaMarcas(InnerModel *inner_) : inner(inner_){
	inMemory=false;
	initMark=0;
}

ListaMarcas::~ListaMarcas()
{
  
}

int ListaMarcas::getInitMark()
{
  return initMark;
}

void ListaMarcas::setInitMark(int id)
{
  initMark=id;
}

bool ListaMarcas::getInMemory()
{
  return inMemory;
}

void ListaMarcas::setInMemory(bool in)
{
  inMemory=in;
}

void ListaMarcas::add(const RoboCompAprilTags::tag &t)
{
  QMutexLocker ml(&mutex);
	
  Marca marca;
  marca.id = t.id;
  marca.rx = t.rx;
  marca.ry = t.ry;
  marca.rz = t.rz;
  marca.tx = t.tx * 1000;
  marca.ty = t.ty * 1000;
  marca.tz = t.tz * 1000;
  marca.reloj=QTime::currentTime();
	
  lista.insert(marca.id, marca);
	
  if(initMark==marca.id)
  {
    memory = inner->transform("world", QVec::vec3(marca.tx, 0, marca.tz),"rgbd");
    inMemory=true;
  }
}
     
ListaMarcas::Marca ListaMarcas::get(int id)
{
  QMutexLocker ml(&mutex);
  if(lista.contains(id))
  {
    return lista.value(id);
  }
  else
  {
    //RECUPERAR memoria
    QVec reality = inner->transform("rgbd", memory ,"world");
    Marca m;
    m.id=id;
    m.tx=reality.x();
    m.ty=reality.y();
    m.tz=reality.z();
    return m;
  }
}
      
bool ListaMarcas::exists(int id)
{
  QMutexLocker ml(&mutex);
  borraMarca(id);

  return lista.contains(id) or inMemory;
}
      
float ListaMarcas::distance(int initMark)
{
  Marca m = get(initMark);
  QMutexLocker ml(&mutex);
  borraMarca(initMark);
  float d = sqrt(pow(m.tx,2) + pow(m.tz,2));
  return d;
}
      
void ListaMarcas::borraMarca(int id)
{
  if(lista.value(id).reloj.elapsed()>300)
    lista.remove(id);
}