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

#ifndef G2O_EDGE_TARGETXYZ_H
#define G2O_EDGE_TARGETXYZ_H

#include "g2o/config.h"
#include "vertex_targetxyz.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"

using namespace Eigen;

namespace g2o {

  class G2O_TYPES_SLAM2D_API EdgeTargetXYZ : public BaseBinaryEdge<9, Eigen::VectorXd, VertexTargetXYZ, VertexTargetXYZ>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeTargetXYZ();

      void computeError()
      {
        const VertexTargetXYZ* m1 = static_cast<const VertexTargetXYZ*>(_vertices[0]); // moving landmark position 1
        const VertexTargetXYZ* m2 = static_cast<const VertexTargetXYZ*>(_vertices[1]); // moving landmark position 2
        //_error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
	
	unsigned long long timestamp2, timestamp1;
	timestamp1 = m1->timestamp;
	timestamp2 = m2->timestamp;
	
	double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
	//std::cout<<deltaT<<std::endl;
	
	MatrixXd F;
	F.resize(9,9);
	F(0, 0) =  1;  F(0, 1) = 0;   F(0, 2) = 0; 	F(0, 3) = deltaT;  	F(0, 4) = 0;		F(0, 5) = 0;		F(0, 6) = 0.5*deltaT*deltaT;	F(0, 7) = 0;			F(0, 8) = 0;
	F(1, 0) =  0;  F(1, 1) = 1;   F(1, 2) = 0; 	F(1, 3) = 0;  		F(1, 4) = deltaT;	F(1, 5) = 0;		F(1, 6) = 0;			F(1, 7) = 0.5*deltaT*deltaT;	F(1, 8) = 0;
	F(2, 0) =  0;  F(2, 1) = 0;   F(2, 2) = 1; 	F(2, 3) = 0;  		F(2, 4) = 0;		F(2, 5) = deltaT;	F(2, 6) = 0;			F(2, 7) = 0;			F(2, 8) = 0.5*deltaT*deltaT;
	F(3, 0) =  0;  F(3, 1) = 0;   F(3, 2) = 0; 	F(3, 3) = 1;  		F(3, 4) = 0;		F(3, 5) = 0;		F(3, 6) = deltaT;		F(3, 7) = 0;			F(3, 8) = 0;
	F(4, 0) =  0;  F(4, 1) = 0;   F(4, 2) = 0; 	F(4, 3) = 0;  		F(4, 4) = 1;		F(4, 5) = 0;		F(4, 6) = 0;			F(4, 7) = deltaT;		F(4, 8) = 0;
	F(5, 0) =  0;  F(5, 1) = 0;   F(5, 2) = 0; 	F(5, 3) = 0;  		F(5, 4) = 0;		F(5, 5) = 1;		F(5, 6) = 0;			F(5, 7) = 0;			F(5, 8) = deltaT;
	F(6, 0) =  0;  F(6, 1) = 0;   F(6, 2) = 0; 	F(6, 3) = 0;  		F(6, 4) = 0;		F(6, 5) = 0;		F(6, 6) = 1;			F(6, 7) = 0;			F(6, 8) = 0;
	F(7, 0) =  0;  F(7, 1) = 0;   F(7, 2) = 0; 	F(7, 3) = 0;  		F(7, 4) = 0;		F(7, 5) = 0;		F(7, 6) = 0;			F(7, 7) = 1;			F(7, 8) = 0;
	F(8, 0) =  0;  F(8, 1) = 0;   F(8, 2) = 0; 	F(8, 3) = 0;  		F(8, 4) = 0;		F(8, 5) = 0;		F(8, 6) = 0;			F(8, 7) = 0;			F(8, 8) = 1;
	_error = (m2->estimate() - F*m1->estimate())- _measurement;

      }

      virtual bool setMeasurementData(const double* d)
      {
	for(int i=0; i<9; i++)
	  _measurement[i]=d[i];
	return true;
      }

      virtual bool getMeasurementData(double* d) const
      {
	for(int i=0; i<9; i++)
	  d[i] = _measurement[i];
	return true;
      }
      
      virtual int measurementDimension() const {return 9;}

      virtual bool setMeasurementFromState()
      {
        const VertexTargetXYZ* m1 = static_cast<const VertexTargetXYZ*>(_vertices[0]);
        const VertexTargetXYZ* m2 = static_cast<const VertexTargetXYZ*>(_vertices[1]);
	
	unsigned long long timestamp2, timestamp1;
	timestamp1 = m1->timestamp;
	timestamp2 = m2->timestamp;
	
	double deltaT = ((double)abs(timestamp2 - timestamp1))/1000000;
	//std::cout<<deltaT<<std::endl;
	
	MatrixXd F;
	F.resize(9,9);
	F(0, 0) =  1;  F(0, 1) = 0;   F(0, 2) = 0; 	F(0, 3) = deltaT;  	F(0, 4) = 0;		F(0, 5) = 0;		F(0, 6) = 0.5*deltaT*deltaT;	F(0, 7) = 0;			F(0, 8) = 0;
	F(1, 0) =  0;  F(1, 1) = 1;   F(1, 2) = 0; 	F(1, 3) = 0;  		F(1, 4) = deltaT;	F(1, 5) = 0;		F(1, 6) = 0;			F(1, 7) = 0.5*deltaT*deltaT;	F(1, 8) = 0;
	F(2, 0) =  0;  F(2, 1) = 0;   F(2, 2) = 1; 	F(2, 3) = 0;  		F(2, 4) = 0;		F(2, 5) = deltaT;	F(2, 6) = 0;			F(2, 7) = 0;			F(2, 8) = 0.5*deltaT*deltaT;
	F(3, 0) =  0;  F(3, 1) = 0;   F(3, 2) = 0; 	F(3, 3) = 1;  		F(3, 4) = 0;		F(3, 5) = 0;		F(3, 6) = deltaT;		F(3, 7) = 0;			F(3, 8) = 0;
	F(4, 0) =  0;  F(4, 1) = 0;   F(4, 2) = 0; 	F(4, 3) = 0;  		F(4, 4) = 1;		F(4, 5) = 0;		F(4, 6) = 0;			F(4, 7) = deltaT;		F(4, 8) = 0;
	F(5, 0) =  0;  F(5, 1) = 0;   F(5, 2) = 0; 	F(5, 3) = 0;  		F(5, 4) = 0;		F(5, 5) = 1;		F(5, 6) = 0;			F(5, 7) = 0;			F(5, 8) = deltaT;
	F(6, 0) =  0;  F(6, 1) = 0;   F(6, 2) = 0; 	F(6, 3) = 0;  		F(6, 4) = 0;		F(6, 5) = 0;		F(6, 6) = 1;			F(6, 7) = 0;			F(6, 8) = 0;
	F(7, 0) =  0;  F(7, 1) = 0;   F(7, 2) = 0; 	F(7, 3) = 0;  		F(7, 4) = 0;		F(7, 5) = 0;		F(7, 6) = 0;			F(7, 7) = 1;			F(7, 8) = 0;
	F(8, 0) =  0;  F(8, 1) = 0;   F(8, 2) = 0; 	F(8, 3) = 0;  		F(8, 4) = 0;		F(8, 5) = 0;		F(8, 6) = 0;			F(8, 7) = 0;			F(8, 8) = 1;	
	_measurement = (m2->estimate() - F*m1->estimate());
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

  class G2O_TYPES_SLAM2D_API EdgeTargetXYZWriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeTargetXYZWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeTargetXYZDrawAction: public DrawAction{
  public:
    EdgeTargetXYZDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
