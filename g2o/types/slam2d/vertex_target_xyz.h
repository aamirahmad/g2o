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

#ifndef G2O_VERTEX_TARGET_XYZ_H
#define G2O_VERTEX_TARGET_XYZ_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"
#include "g2o_types_slam2d_api.h"

using namespace Eigen;

namespace g2o 
{

  /**
   * \brief moving spherical landmark (no orientation) position (3D) + velocity(3D) + Acceleration(3D) hence a 4D Vertex, (x,y,z,Vx,Vy,Vz,Ax,Ay,Az)
   */
  class G2O_TYPES_SLAM2D_API VertexTarget_XYZ : public BaseVertex<9, Eigen::VectorXd>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VertexTarget_XYZ();

      virtual void setToOriginImpl() 
      {
        _estimate.setZero();
      }

      virtual bool setEstimateDataImpl(const double* est)
      {
	for(int i=0; i<9; i++)
	  _estimate[i] = est[i];
	return true;
      }

      virtual bool getEstimateData(double* est) const
      {
	for(int i=0; i<9; i++)
	  est[i] = _estimate[i];
	return true;
      }
	  
      virtual int estimateDimension() const 
      { 
	return 9;
      }

      virtual bool setMinimalEstimateDataImpl(const double* est)
      {
	return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(double* est) const
      {
	return getEstimateData(est);
      }
	  
      virtual int minimalEstimateDimension() const 
      { 
	return 9;
      }

      virtual void oplusImpl(const double* update)
      {
	
        for(int i=0; i<9; i++)
	  _estimate[i] += (update[i]);
	  //_estimate[i] += ((update[i])==(update[i]) ? (update[i]) : 0);

	
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
    
      unsigned long long timestamp;
      bool isOptimizedAtLeastOnce;

  };

  class G2O_TYPES_SLAM2D_API VertexTarget_XYZWriteGnuplotAction: public WriteGnuplotAction 
  {
    public:
	VertexTarget_XYZWriteGnuplotAction();
	virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API VertexTarget_XYZDrawAction: public DrawAction
  {
      public:
	VertexTarget_XYZDrawAction();
	virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
      protected:
	FloatProperty *_pointSize;
	virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };
#endif

}

#endif
