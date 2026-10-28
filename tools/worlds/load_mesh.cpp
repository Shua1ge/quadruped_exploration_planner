#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <fstream>
#include <iostream>
#include <limits>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 const auto*m=ignition::common::MeshManager::Instance()->Load(argv[1]);
 if(!m){std::cerr<<"MESH_LOAD_FAILED\n";return 3;}
 std::ofstream out(argv[2],std::ios::binary);
 size_t triangles=0,nontriangle=0,invalid=0;
 for(unsigned s=0;s<m->SubMeshCount();++s){
  auto sm=m->SubMeshByIndex(s).lock();
  std::cerr<<"SUBMESH "<<s<<" "<<sm->Name()<<" vertices="<<sm->VertexCount()<<" indices="<<sm->IndexCount()<<" primitive="<<sm->SubMeshPrimitiveType()<<"\n";
  if(sm->SubMeshPrimitiveType()!=ignition::common::SubMesh::TRIANGLES){++nontriangle;continue;}
  for(unsigned i=0;i+2<sm->IndexCount();i+=3){
   float tri[9];bool ok=true;
   for(unsigned j=0;j<3;++j){int k=sm->Index(i+j);if(k<0||unsigned(k)>=sm->VertexCount()){ok=false;break;}
    auto v=sm->Vertex(k);for(unsigned a=0;a<3;++a)tri[j*3+a]=v[a];}
   if(ok){out.write(reinterpret_cast<const char*>(tri),sizeof(tri));++triangles;}else ++invalid;
  }
 }
 std::cout<<"{\"submeshes\":"<<m->SubMeshCount()<<",\"vertices\":"<<m->VertexCount()<<",\"indices\":"<<m->IndexCount()<<",\"triangles\":"<<triangles<<",\"nontriangle_submeshes\":"<<nontriangle<<",\"invalid_triangles\":"<<invalid<<"}\n";
 return 0;
}
