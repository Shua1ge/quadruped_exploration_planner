#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/config.h>
#include <assimp/scene.h>
#include <iostream>
#include <chrono>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 auto start=std::chrono::steady_clock::now();
 Assimp::Importer importer;
 // Gazebo's Collada loader preserves this source's Z_UP world coordinates.
 // Assimp otherwise normalizes Z_UP to Y_UP before baking node transforms;
 // OBJ has no up-axis metadata to undo that conversion at runtime.
 importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
 importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE,aiPrimitiveType_POINT|aiPrimitiveType_LINE);
 const auto*scene=importer.ReadFile(argv[1],aiProcess_Triangulate|aiProcess_PreTransformVertices|aiProcess_JoinIdenticalVertices|aiProcess_RemoveRedundantMaterials|aiProcess_SortByPType|aiProcess_OptimizeMeshes);
 if(!scene){std::cerr<<importer.GetErrorString()<<"\n";return 3;}
 size_t vertices=0,faces=0;for(unsigned i=0;i<scene->mNumMeshes;++i){vertices+=scene->mMeshes[i]->mNumVertices;faces+=scene->mMeshes[i]->mNumFaces;}
 Assimp::Exporter exporter;if(exporter.Export(scene,"obj",argv[2])!=AI_SUCCESS){std::cerr<<exporter.GetErrorString()<<"\n";return 4;}
 std::cout<<"{\"meshes\":"<<scene->mNumMeshes<<",\"materials\":"<<scene->mNumMaterials<<",\"vertices\":"<<vertices<<",\"faces\":"<<faces<<",\"elapsed_wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"}\n";
}
