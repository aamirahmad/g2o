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

#ifndef G2O_EDGE_PTXYZ_TARGETXYZ_H
#define G2O_EDGE_PTXYZ_TARGETXYZ_H

#include "g2o/config.h"
#include "vertex_pointxyz.h"
#include "vertex_targetxyz.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"

using namespace Eigen;

namespace g2o {

  class G2O_TYPES_SLAM2D_API EdgePtXYZ_TargetXYZ : public BaseBinaryEdge<3, Eigen::Vector3d, VertexPtXYZ, VertexTargetXYZ>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgePtXYZ_TargetXYZ();

      void computeError()
      {
        const VertexPtXYZ* v1 = static_cast<const VertexPtXYZ*>(_vertices[0]);
        const VertexTargetXYZ* l2 = static_cast<const VertexTargetXYZ*>(_vertices[1]);
	
	Vector3d pos1(v1->estimate()(0),v1->estimate()(1),v1->estimate()(2));
	Vector3d pos2(l2->estimate()(0),l2->estimate()(1),l2->estimate()(2));
	
        _error = (pos2-pos1) - _measurement; // expeted - measured
      }

      virtual bool setMeasurementData(const double* d)
      {
	_measurement[0]=d[0];
	_measurement[1]=d[1];
	_measurement[2]=d[2];
	return true;
      }

      virtual bool getMeasurementData(double* d) const
      {
	d[0] = _measurement[0];
	d[1] = _measurement[1];
	d[2] = _measurement[2];
	return true;
      }
      
      virtual int measurementDimension() const {return 3;}

      virtual bool setMeasurementFromState()
      {
        const VertexPtXYZ* v1 = static_cast<const VertexPtXYZ*>(_vertices[0]);
        const VertexTargetXYZ* l2 = static_cast<const VertexTargetXYZ*>(_vertices[1]);

	Vector3d pos1(v1->estimate()(0),v1->estimate()(1),v1->estimate()(2));
	Vector3d pos2(l2->estimate()(0),l2->estimate()(1),l2->estimate()(2));
	
	_measurement = pos2-pos1;
	return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif
  };

  class G2O_TYPES_SLAM2D_API EdgePtXYZ_TargetXYZWriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgePtXYZ_TargetXYZWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgePtXYZ_TargetXYZDrawAction: public DrawAction{
  public:
    EdgePtXYZ_TargetXYZDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
