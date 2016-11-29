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

#include "vertex_target_xyz.h"
#include <typeinfo>

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

  VertexTarget_XYZ::VertexTarget_XYZ() : BaseVertex<9, VectorXd>()
  {
    _estimate.setZero();
  }

  bool VertexTarget_XYZ::read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1] >> _estimate[2] >> _estimate[3] >> _estimate[4] >> _estimate[5] >> _estimate[6] >> _estimate[7] >> _estimate[8] >> timestamp;
    return true;
  }

  bool VertexTarget_XYZ::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2)<< " " << estimate()(3)<< " " << estimate()(4) << " " << estimate()(5) << " " << estimate()(6) << " " << estimate()(7) << " " <<estimate()(8) << " " << timestamp;
    return os.good();
  }

  VertexTarget_XYZWriteGnuplotAction::VertexTarget_XYZWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexTarget_XYZ).name()){}

  HyperGraphElementAction* VertexTarget_XYZWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_)
  {
    if (typeid(*element).name()!=_typeName)
      return 0;

    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os)
    {
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return false;
    }
         
    VertexTarget_XYZ* v =  static_cast<VertexTarget_XYZ*>(element);
    *(params->os) << v->estimate()(0) << " " << v->estimate()(1) << " " << v->estimate()(2) << " " << v->estimate()(3) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  VertexTarget_XYZDrawAction::VertexTarget_XYZDrawAction(): DrawAction(typeid(VertexTarget_XYZ).name())
  {
  }

  bool VertexTarget_XYZDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_)
  {
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams)
    {
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } 
    else 
    {
      _pointSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexTarget_XYZDrawAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ )
  {
    if (typeid(*element).name()!=_typeName)
      return 0;

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;
   
    VertexTarget_XYZ* that = static_cast<VertexTarget_XYZ*>(element);
    glColor3f(0.2f,0.5f,0.3f);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    glBegin(GL_POINTS);
    glVertex3f((float)that->estimate()(0),(float)that->estimate()(1),0.f);
    glEnd();
    return this;
  }
#endif

} // end namespace
