// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_pointxyz_targetxyz.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace Eigen;

namespace g2o {

  EdgePtXYZ_TargetXYZ::EdgePtXYZ_TargetXYZ() :
    BaseBinaryEdge<3, Eigen::Vector3d, VertexPtXYZ, VertexTargetXYZ>()
  {
  }

  bool EdgePtXYZ_TargetXYZ::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1] >> _measurement[2];
    
    is >> information()(0,0) >> information()(0,1) >> information()(0,2) >> information()(1,1) >> information()(1,2) >> information()(2,2);
    
    information()(1,0) = information()(0,1);
    information()(2,0) = information()(0,2);
    information()(2,1) = information()(1,2);
    
    return true;
  }

  bool EdgePtXYZ_TargetXYZ::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " " << measurement()[2] << " ";
    
    os << information()(0,0) << " " << information()(0,1) << " " << information()(0,2) << " " << information()(1,1) << " " << information()(1,2) << " " << information()(2,2);
    
    return os.good();
  }

  void EdgePtXYZ_TargetXYZ::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexSE2 position by VertexXY_VXVY");

    VertexPtXYZ* vi  = static_cast<VertexPtXYZ*>(_vertices[0]);
    VertexTargetXYZ* vj = static_cast<VertexTargetXYZ*>(_vertices[1]);
  
    if (from.count(vi) > 0 && to == vj) 
    {  
      Vector3d newPos((vi->estimate() + _measurement));
      vj->setEstimate(newPos);
    }
  }

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgePtXYZ_TargetXYZ::linearizeOplus()
  {

    _jacobianOplusXi( 0 , 0 ) = -1;
    _jacobianOplusXi( 0 , 1 ) = 0;
    _jacobianOplusXi( 0 , 2 ) = 0;
    _jacobianOplusXi( 1 , 0 ) = 0;
    _jacobianOplusXi( 1 , 1 ) = -1;
    _jacobianOplusXi( 1 , 2 ) = 0;
    _jacobianOplusXi( 2 , 0 ) = 0;
    _jacobianOplusXi( 2 , 1 ) = 0;
    _jacobianOplusXi( 2 , 2 ) = -1;    

    _jacobianOplusXj( 0 , 0 ) = 1;
    _jacobianOplusXj( 0 , 1 ) = 0;
    _jacobianOplusXj( 0 , 2 ) = 0;
    _jacobianOplusXj( 1 , 0 ) = 0;
    _jacobianOplusXj( 1 , 1 ) = 1;
    _jacobianOplusXj( 1 , 2 ) = 0;
    _jacobianOplusXj( 2 , 0 ) = 0;
    _jacobianOplusXj( 2 , 1 ) = 0;
    _jacobianOplusXj( 2 , 2 ) = 1;       

  }
#endif

  EdgePtXYZ_TargetXYZWriteGnuplotAction::EdgePtXYZ_TargetXYZWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgePtXYZ_TargetXYZ).name()){}

  HyperGraphElementAction* EdgePtXYZ_TargetXYZWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgePtXYZ_TargetXYZ* e =  static_cast<EdgePtXYZ_TargetXYZ*>(element);
    VertexPtXYZ* fromEdge = static_cast<VertexPtXYZ*>(e->vertex(0));
    VertexTargetXYZ* toEdge   = static_cast<VertexTargetXYZ*>(e->vertex(1));
    *(params->os) << fromEdge->estimate()(0) << " " << fromEdge->estimate()(1)
      << " " << fromEdge->estimate()(2) << std::endl;
    *(params->os) << toEdge->estimate()(0) << " " << toEdge->estimate()(1) << " " << toEdge->estimate()(2) << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgePtXYZ_TargetXYZDrawAction::EdgePtXYZ_TargetXYZDrawAction(): DrawAction(typeid(EdgePtXYZ_TargetXYZ).name()){}

  HyperGraphElementAction* EdgePtXYZ_TargetXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters*  params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;


    EdgePtXYZ_TargetXYZ* e =  static_cast<EdgePtXYZ_TargetXYZ*>(element);
    VertexPtXYZ* fromEdge = static_cast<VertexPtXYZ*>(e->vertex(0));
    VertexTargetXYZ* toEdge   = static_cast<VertexTargetXYZ*>(e->vertex(1));
    glColor3f(0.4f,0.4f,0.2f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate()(0),(float)fromEdge->estimate()(1),0.f);
    glVertex3f((float)toEdge->estimate()(0),(float)toEdge->estimate()(1),0.f);
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
