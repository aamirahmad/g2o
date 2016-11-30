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

#include "edge_targetxyz.h"

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

  EdgeTargetXYZ::EdgeTargetXYZ() :
    BaseBinaryEdge<9, VectorXd, VertexTargetXYZ, VertexTargetXYZ>()
  {
  }

  bool EdgeTargetXYZ::read(std::istream& is)
  {
    is >> _measurement[0] >> _measurement[1] >> _measurement[2] >> _measurement[3] >> _measurement[4] >> _measurement[5] >> _measurement[6] >> _measurement[7] >> _measurement[8];
    
    is >> information()(0,0) >> information()(0,1) >> information()(0,2) >> information()(0,3) >> information()(0,4) >> information()(0,5) >> information()(0,6) >> information()(0,7) >> information()(0,8) >> 
    information()(1,1) >> information()(1,2) >> information()(1,3) >> information()(1,4) >> information()(1,5) >> information()(1,6) >> information()(1,7) >> information()(1,8) >>
    information()(2,2) >> information()(2,3) >> information()(2,4) >> information()(2,5) >> information()(2,6) >> information()(2,7) >> information()(2,8) >>
    information()(3,3) >> information()(3,4) >> information()(3,5) >> information()(3,6) >> information()(3,7) >> information()(3,8) >>
    information()(4,4) >> information()(4,5) >> information()(4,6) >> information()(4,7) >> information()(4,8) >>
    information()(5,5) >> information()(5,6) >> information()(5,7) >> information()(5,8) >>
    information()(6,6) >> information()(6,7) >> information()(6,8) >>
    information()(7,7) >> information()(7,8) >>
    information()(8,8);
    
    for(int i=1; i<9; i++)
      for (int j=0; j<i; j++)
	information()(i,j) = information()(j,i);

    return true;
  }

  bool EdgeTargetXYZ::write(std::ostream& os) const
  {
    os << measurement()[0] << " " << measurement()[1] << " " << measurement()[2] << " " << measurement()[3] << " " << measurement()[4] << " " << measurement()[5] << " " << measurement()[6] << " " << measurement()[7] << " " << measurement()[8] << " ";
    os << information()(0,0) << " " << information()(0,1) << " " << information()(0,2) << " " << information()(0,3) << " " << information()(0,4) << " " << information()(0,5) << " " << information()(0,6) << " " << information()(0,7) << " " << information()(0,8) << " " << 
    information()(1,1) << " " << information()(1,2) << " " << information()(1,3) << " " << information()(1,4) << " " << information()(1,5) << " " << information()(1,6) << " " << information()(1,7) << " " << information()(1,8) << " " <<
    information()(2,2) << " " << information()(2,3) << " " << information()(2,4) << " " << information()(2,5) << " " << information()(2,6) << " " << information()(2,7) << " " << information()(2,8) << " " <<
    information()(3,3) << " " << information()(3,4) << " " << information()(3,5) << " " << information()(3,6) << " " << information()(3,7) << " " << information()(3,8) << " " <<
    information()(4,4) << " " << information()(4,5) << " " << information()(4,6) << " " << information()(4,7) << " " << information()(4,8) << " " <<
    information()(5,5) << " " << information()(5,6) << " " << information()(5,7) << " " << information()(5,8) << " " <<
    information()(6,6) << " " << information()(6,7) << " " << information()(6,8) << " " <<
    information()(7,7) << " " << information()(7,8) << " " <<
    information()(8,8);
    
    return os.good();
  }

  void EdgeTargetXYZ::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexTargetXYZ position by VertexTargetXYZ");

    VertexTargetXYZ* vi = static_cast<VertexTargetXYZ*>(_vertices[0]);
    VertexTargetXYZ* vj = static_cast<VertexTargetXYZ*>(_vertices[1]);
    if (from.count(vi) > 0 && to == vj) 
    {
      // Don't worry much of the magic number below. It is just to initialize!
      VectorXd temp(9);
      temp<<
	vi->estimate()(0) + _measurement[0] + _measurement[3]*0.033 + _measurement[6]*0.033*0.033,
	vi->estimate()(1) + _measurement[1] + _measurement[4]*0.033 + _measurement[7]*0.033*0.033,
	vi->estimate()(2) + _measurement[2] + _measurement[5]*0.033 + _measurement[8]*0.033*0.033,
	vi->estimate()(3) + _measurement[3] + _measurement[4]*0.033,
	vi->estimate()(4) + _measurement[4] + _measurement[5]*0.033,
	vi->estimate()(5) + _measurement[5] + _measurement[6]*0.033,
        vi->estimate()(6) + _measurement[6],
	vi->estimate()(7) + _measurement[7],		       
	vi->estimate()(8) + _measurement[8];
      vj->setEstimate(temp);
    }
  }
/*
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void EdgeTargetXYZ::linearizeOplus()
  {
      const VertexTargetXYZ* vi = static_cast<const VertexTargetXYZ*>(_vertices[0]);
      const VertexTargetXYZ* vj = static_cast<const VertexTargetXYZ*>(_vertices[1]);
      
      unsigned long long timestamp2, timestamp1;
      timestamp1 = vi->timestamp;
      timestamp2 = vj->timestamp;
      
      double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
      //std::cout<<deltaT<<std::endl;      
      
	_jacobianOplusXi(0, 0) =  -1;  _jacobianOplusXi(0, 1) = 0;   _jacobianOplusXi(0, 2) = 0; 	_jacobianOplusXi(0, 3) = -deltaT;  	_jacobianOplusXi(0, 4) = 0;		_jacobianOplusXi(0, 5) = 0;		_jacobianOplusXi(0, 6) = -0.5*deltaT*deltaT;	_jacobianOplusXi(0, 7) = 0;			_jacobianOplusXi(0, 8) = 0;
	_jacobianOplusXi(1, 0) =  0;  _jacobianOplusXi(1, 1) = -1;   _jacobianOplusXi(1, 2) = 0; 	_jacobianOplusXi(1, 3) = 0;  		_jacobianOplusXi(1, 4) = -deltaT;	_jacobianOplusXi(1, 5) = 0;		_jacobianOplusXi(1, 6) = 0;			_jacobianOplusXi(1, 7) = -0.5*deltaT*deltaT;	_jacobianOplusXi(1, 8) = 0;
	_jacobianOplusXi(2, 0) =  0;  _jacobianOplusXi(2, 1) = 0;   _jacobianOplusXi(2, 2) = -1; 	_jacobianOplusXi(2, 3) = 0;  		_jacobianOplusXi(2, 4) = 0;		_jacobianOplusXi(2, 5) = -deltaT;	_jacobianOplusXi(2, 6) = 0;			_jacobianOplusXi(2, 7) = 0;			_jacobianOplusXi(2, 8) = -0.5*deltaT*deltaT;
	_jacobianOplusXi(3, 0) =  0;  _jacobianOplusXi(3, 1) = 0;   _jacobianOplusXi(3, 2) = 0; 	_jacobianOplusXi(3, 3) = -1;  		_jacobianOplusXi(3, 4) = 0;		_jacobianOplusXi(3, 5) = 0;		_jacobianOplusXi(3, 6) = -deltaT;		_jacobianOplusXi(3, 7) = 0;			_jacobianOplusXi(3, 8) = 0;
	_jacobianOplusXi(4, 0) =  0;  _jacobianOplusXi(4, 1) = 0;   _jacobianOplusXi(4, 2) = 0; 	_jacobianOplusXi(4, 3) = 0;  		_jacobianOplusXi(4, 4) = -1;		_jacobianOplusXi(4, 5) = 0;		_jacobianOplusXi(4, 6) = 0;			_jacobianOplusXi(4, 7) = -deltaT;		_jacobianOplusXi(4, 8) = 0;
	_jacobianOplusXi(5, 0) =  0;  _jacobianOplusXi(5, 1) = 0;   _jacobianOplusXi(5, 2) = 0; 	_jacobianOplusXi(5, 3) = 0;  		_jacobianOplusXi(5, 4) = 0;		_jacobianOplusXi(5, 5) = -1;		_jacobianOplusXi(5, 6) = 0;			_jacobianOplusXi(5, 7) = 0;			_jacobianOplusXi(5, 8) = -deltaT;
	_jacobianOplusXi(6, 0) =  0;  _jacobianOplusXi(6, 1) = 0;   _jacobianOplusXi(6, 2) = 0; 	_jacobianOplusXi(6, 3) = 0;  		_jacobianOplusXi(6, 4) = 0;		_jacobianOplusXi(6, 5) = 0;		_jacobianOplusXi(6, 6) = -1;			_jacobianOplusXi(6, 7) = 0;			_jacobianOplusXi(6, 8) = 0;
	_jacobianOplusXi(7, 0) =  0;  _jacobianOplusXi(7, 1) = 0;   _jacobianOplusXi(7, 2) = 0; 	_jacobianOplusXi(7, 3) = 0;  		_jacobianOplusXi(7, 4) = 0;		_jacobianOplusXi(7, 5) = 0;		_jacobianOplusXi(7, 6) = 0;			_jacobianOplusXi(7, 7) = -1;			_jacobianOplusXi(7, 8) = 0;
	_jacobianOplusXi(8, 0) =  0;  _jacobianOplusXi(8, 1) = 0;   _jacobianOplusXi(8, 2) = 0; 	_jacobianOplusXi(8, 3) = 0;  		_jacobianOplusXi(8, 4) = 0;		_jacobianOplusXi(8, 5) = 0;		_jacobianOplusXi(8, 6) = 0;			_jacobianOplusXi(8, 7) = 0;			_jacobianOplusXi(8, 8) = -1;	    

	_jacobianOplusXj(0, 0) =  1;  _jacobianOplusXj(0, 1) = 0;   _jacobianOplusXj(0, 2) = 0; 	_jacobianOplusXj(0, 3) = 0;  		_jacobianOplusXj(0, 4) = 0;		_jacobianOplusXj(0, 5) = 0;		_jacobianOplusXj(0, 6) = 0;			_jacobianOplusXj(0, 7) = 0;			_jacobianOplusXj(0, 8) = 0;
	_jacobianOplusXj(1, 0) =  0;  _jacobianOplusXj(1, 1) = 1;   _jacobianOplusXj(1, 2) = 0; 	_jacobianOplusXj(1, 3) = 0;  		_jacobianOplusXj(1, 4) = 0;		_jacobianOplusXj(1, 5) = 0;		_jacobianOplusXj(1, 6) = 0;			_jacobianOplusXj(1, 7) = 0;			_jacobianOplusXj(1, 8) = 0;
	_jacobianOplusXj(2, 0) =  0;  _jacobianOplusXj(2, 1) = 0;   _jacobianOplusXj(2, 2) = 1; 	_jacobianOplusXj(2, 3) = 0;  		_jacobianOplusXj(2, 4) = 0;		_jacobianOplusXj(2, 5) = 0;		_jacobianOplusXj(2, 6) = 0;			_jacobianOplusXj(2, 7) = 0;			_jacobianOplusXj(2, 8) = 0;
	_jacobianOplusXj(3, 0) =  0;  _jacobianOplusXj(3, 1) = 0;   _jacobianOplusXj(3, 2) = 0; 	_jacobianOplusXj(3, 3) = 1;  		_jacobianOplusXj(3, 4) = 0;		_jacobianOplusXj(3, 5) = 0;		_jacobianOplusXj(3, 6) = 0;			_jacobianOplusXj(3, 7) = 0;			_jacobianOplusXj(3, 8) = 0;
	_jacobianOplusXj(4, 0) =  0;  _jacobianOplusXj(4, 1) = 0;   _jacobianOplusXj(4, 2) = 0; 	_jacobianOplusXj(4, 3) = 0;  		_jacobianOplusXj(4, 4) = 1;		_jacobianOplusXj(4, 5) = 0;		_jacobianOplusXj(4, 6) = 0;			_jacobianOplusXj(4, 7) = 0;			_jacobianOplusXj(4, 8) = 0;
	_jacobianOplusXj(5, 0) =  0;  _jacobianOplusXj(5, 1) = 0;   _jacobianOplusXj(5, 2) = 0; 	_jacobianOplusXj(5, 3) = 0;  		_jacobianOplusXj(5, 4) = 0;		_jacobianOplusXj(5, 5) = 1;		_jacobianOplusXj(5, 6) = 0;			_jacobianOplusXj(5, 7) = 0;			_jacobianOplusXj(5, 8) = 0;
	_jacobianOplusXj(6, 0) =  0;  _jacobianOplusXj(6, 1) = 0;   _jacobianOplusXj(6, 2) = 0; 	_jacobianOplusXj(6, 3) = 0;  		_jacobianOplusXj(6, 4) = 0;		_jacobianOplusXj(6, 5) = 0;		_jacobianOplusXj(6, 6) = 1;			_jacobianOplusXj(6, 7) = 0;			_jacobianOplusXj(6, 8) = 0;
	_jacobianOplusXj(7, 0) =  0;  _jacobianOplusXj(7, 1) = 0;   _jacobianOplusXj(7, 2) = 0; 	_jacobianOplusXj(7, 3) = 0;  		_jacobianOplusXj(7, 4) = 0;		_jacobianOplusXj(7, 5) = 0;		_jacobianOplusXj(7, 6) = 0;			_jacobianOplusXj(7, 7) = 1;			_jacobianOplusXj(7, 8) = 0;
	_jacobianOplusXj(8, 0) =  0;  _jacobianOplusXj(8, 1) = 0;   _jacobianOplusXj(8, 2) = 0; 	_jacobianOplusXj(8, 3) = 0;  		_jacobianOplusXj(8, 4) = 0;		_jacobianOplusXj(8, 5) = 0;		_jacobianOplusXj(8, 6) = 0;			_jacobianOplusXj(8, 7) = 0;			_jacobianOplusXj(8, 8) = 1;	
  
  }
#endif*/

  EdgeTargetXYZWriteGnuplotAction::EdgeTargetXYZWriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeTargetXYZ).name()){}

  HyperGraphElementAction* EdgeTargetXYZWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_)
  {
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeTargetXYZ* e =  static_cast<EdgeTargetXYZ*>(element);
    VertexTargetXYZ* fromEdge = static_cast<VertexTargetXYZ*>(e->vertex(0));
    VertexTargetXYZ* toEdge   = static_cast<VertexTargetXYZ*>(e->vertex(1));
    *(params->os) << fromEdge->estimate()(0) << " " << fromEdge->estimate()(1) << " "<< fromEdge->estimate()(2) << " " << fromEdge->estimate()(3) << std::endl;
    *(params->os) << toEdge->estimate()(0) << " " << toEdge->estimate()(1) << toEdge->estimate()(2) << " " << toEdge->estimate()(3) << std::endl;
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeTargetXYZDrawAction::EdgeTargetXYZDrawAction(): DrawAction(typeid(EdgeTargetXYZ).name()){}

  HyperGraphElementAction* EdgeTargetXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters*  params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;


    EdgeTargetXYZ* e =  static_cast<EdgeTargetXYZ*>(element);
    VertexTargetXYZ* fromEdge = static_cast<VertexTargetXYZ*>(e->vertex(0));
    VertexTargetXYZ* toEdge   = static_cast<VertexTargetXYZ*>(e->vertex(1));
    glColor3f(0.4f,0.1f,0.8f);
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
