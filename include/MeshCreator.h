/**
 * @file MeshCreator.h
 * @brief This class creates the mesh from the point cloud and normals. Possion surface reconstruction algorithm needs to be implemented here.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
//TODO: Whole
#ifndef MESH_CREATOR_H
#define MESH_CREATOR_H

namespace reconstruction{

class MeshCreator
{
private:
    /* data */
public:
    MeshCreator(/* args */);
    ~MeshCreator();
    void createMesh();
};

};

#endif //MESH_CREATOR_H