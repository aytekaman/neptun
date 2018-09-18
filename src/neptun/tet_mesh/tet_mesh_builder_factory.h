#pragma once

#include <cassert>
#include <map>
#include <memory>
#include <string>

#include "tet_mesh_builder.h"
#include "tetgen_tet_mesh_builder.h"

// Singleton class that stores all of the available TetMesh builders 
// NOTE: Available tetmesh builders must be registered inside initialize function
// Use TetMeshBuilderFactory::builders() to get the all of the builders
// Use TetMeshBuilderFactory::builder(std::string name) to get specific builder
class TetMeshBuilderFactory{
public:
    using BuilderMapType = std::map< std::string, std::unique_ptr< TetMeshBuilder > >;

    static TetMeshBuilderFactory& instance(){
        static TetMeshBuilderFactory i;
        return i;
    }
    
    static BuilderMapType& builders(){
        return TetMeshBuilderFactory::instance().m_builders;
    }

    static TetMeshBuilder* builder(std::string builder_name){
        BuilderMapType&  builders = TetMeshBuilderFactory::builders();
        auto b = builders.find(builder_name);
        if (b == builders.end())
            return nullptr;
        else
            return b->second.get();
    }

    static TetMeshBuilder* default_builder(){
        return TetMeshBuilderFactory::instance().m_default_builder;
    }

    BuilderMapType& get_builders(){
        return m_builders;
    }

    // delete copy and move constructors and assign operators
    TetMeshBuilderFactory(TetMeshBuilderFactory const &) = delete;            // Copy construct
    TetMeshBuilderFactory(TetMeshBuilderFactory &&) = delete;                 // Move construct
    TetMeshBuilderFactory &operator=(TetMeshBuilderFactory const &) = delete; // Copy assign
    TetMeshBuilderFactory &operator=(TetMeshBuilderFactory &&) = delete;      // Move assign
private:
    inline void register_builder(std::string name, TetMeshBuilder *builder){
        assert(m_builders.find(name) == m_builders.end());
        m_builders.emplace(name, std::unique_ptr<TetMeshBuilder>(builder));
    }

    TetMeshBuilderFactory(){
        initialize_builders();
    }    

    void initialize_builders(){
        TetgenTetMeshBuilder* tetgen_builder = new TetgenTetMeshBuilder();
        register_builder("TetGen", tetgen_builder);

        m_default_builder = tetgen_builder;
    }

    BuilderMapType m_builders;
    TetMeshBuilder* m_default_builder;
};
