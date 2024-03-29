#include <Eigen/Core>
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include "../ceres/autodiff.h"

#include "../tools/rotation.h"
#include "../common/projection.h"

class VertexCameraBAL : public g2o::BaseVertex<9, 
											Eigen::VectorXd>{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexCameraBAL(){}

	virtual bool read (std::istream&){
		return false;
	}

	virtual bool write (std::ostream&)const{
		return false;
	}
	
	virtual void setToOriginImpl(){}

	virtual void oplusImpl(const double* update){
		Eigen::VectorXd::ConstMapType v(update, 
									VertexCameraBAL::Dimension);
		_estimate += v;
	}
};

class VertexPointBAL : public g2o::BaseVertex<3, 
											Eigen::Vector3d>{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexPointBAL(){}
	virtual bool read (std::istream&){
		return false;
	}

	virtual bool write (std::ostream&)const{
		return false;
	}
	
	virtual void setToOriginImpl(){}

	virtual void oplusImpl(const double* update){
		Eigen::Vector3d::ConstMapType v(update);
		_estimate += v;
	}
	
};


class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, 
			Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeObservationBAL(){}
	virtual bool read (std::istream&){
		return false;
	}

	virtual bool write (std::ostream&)const{
		return false;
	}
		
	virtual void computeError() override{
		const VertexCameraBAL* cam = static_cast<const 
					VertexCameraBAL*>(vertex(0));
		const VertexPointBAL* point = static_cast<const
					VertexPointBAL*>(vertex(1));
		(*this)(cam->estimate().data(), 
				point->estimate().data(), _error.data());			
	}

	template<typename T>
	bool operator()(const T* camera, const T* point, T* residuals) const{
		T predictions[2];
		CamProjectionWithDistortion(camera, point, predictions);
		residuals[0]=predictions[0]-T(measurement()(0));
		residuals[1]=predictions[1]-T(measurement()(1));

		return true;
	}

	virtual void linearizeOplus() override{
		const VertexCameraBAL* cam = static_cast<const 
					VertexCameraBAL*>(vertex(0));
		const VertexPointBAL* point = static_cast<const
					VertexPointBAL*>(vertex(1));
					
		typedef ceres::internal::AutoDiff<EdgeObservationBAL,
						double, VertexCameraBAL::Dimension, 
						VertexPointBAL::Dimension> BalAutoDiff;			
	
		Eigen::Matrix<double, Dimension,
		VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
		Eigen::Matrix<double, Dimension,
		VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;

		double *parameters[] = { const_cast<double*>(cam->estimate().data()), 
			const_cast<double*>(point->estimate().data())};
		double *jacobians[] = {dError_dCamera.data(), dError_dPoint.data()};
		double value[Dimension];
		bool diffState = BalAutoDiff::Differentiate(*this, parameters, 
			Dimension, value, jacobians);

		if (diffState){

			_jacobianOplusXi = dError_dCamera;
			_jacobianOplusXj = dError_dPoint;
		}
		else{

			assert(0 && "Error while differentiating");
			_jacobianOplusXi.setZero();
			_jacobianOplusXj.setZero();

		}
	}

};