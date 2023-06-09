/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <cstring>
#include <assert.h>

#include "any_rbdl/rbdl_mathutils.h"
#include "any_rbdl/Logging.h"

#include "any_rbdl/Model.h"
#include "any_rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;
// NOTE: Adapted for mimic
ANY_RBDL_DLLAPI
void UpdateKinematics (Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		bool gravity
		) {
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	if(gravity){
	  SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);
	  model.a[0] = -spatial_gravity;
	}else{
	  model.a[0].setZero();
	}

	for (i = 1; i < model.mBodies.size(); i++) {

		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

		model.X_lambda[i] = model.X_J[i] * model.X_T[i];

		if (lambda != 0) {
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
		}	else {
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = model.v_J[i];
		}

		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

		double qddot0 = joint.mMimicMult * QDDot[joint.q_index];

		if (model.mJoints[i].mDoFCount == 3) {
		double qddot1 = joint.mMimicMult * QDDot[joint.q_index+1];
		double qddot2 = joint.mMimicMult * QDDot[joint.q_index+2];
			Vector3d omegadot_temp (qddot0, qddot1, qddot2);
			model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
		} else {
			model.a[i] = model.a[i] + model.S[i] * qddot0;
		}
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		ANY_RBDL_LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}
}
// NOTE: Adapted for mimic
ANY_RBDL_DLLAPI
void UpdateKinematicsCustom (Model &model,
		const VectorNd *Q,
		const VectorNd *QDot,
		const VectorNd *QDDot
		) {
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	if (Q) {
		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int lambda = model.lambda[i];

			VectorNd QDot_zero (VectorNd::Zero (model.dof_count));

			jcalc (model, i, (*Q), QDot_zero);

			model.X_lambda[i] = model.X_J[i] * model.X_T[i];

			if (lambda != 0) {
				model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			}	else {
				model.X_base[i] = model.X_lambda[i];
			}
		}
	}

	if (QDot) {
		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int lambda = model.lambda[i];

			jcalc (model, i, *Q, *QDot);

			if (lambda != 0) {
				model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
				model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
			}	else {
				model.v[i] = model.v_J[i];
				model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
			}
			// ANY_RBDL_LOG << "v[" << i << "] = " << model.v[i].transpose() << std::endl;
		}
	}

	if (QDDot) {
		for (i = 1; i < model.mBodies.size(); i++) {
			Joint joint = model.mJoints[i];

			unsigned int lambda = model.lambda[i];

			if (lambda != 0) {
				model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
			}	else {
				model.a[i] = model.c[i];
			}

			double qddot0 = joint.mMimicMult * (*QDDot)[joint.q_index];

			if (model.mJoints[i].mDoFCount == 3) {
				double qddot1 = joint.mMimicMult * (*QDDot)[joint.q_index+1];
				double qddot2 = joint.mMimicMult * (*QDDot)[joint.q_index+2];
				Vector3d omegadot_temp (qddot0, qddot1, qddot2);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			} else {
				model.a[i] = model.a[i] + model.S[i] * qddot0;
			}
		}
	}
}

ANY_RBDL_DLLAPI
Vector3d CalcBodyToBaseCoordinates (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_body_coordinates,
		bool update_kinematics) {
	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	if (body_id >= model.fixed_body_discriminator) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		unsigned int parent_id = model.mFixedBodies[fbody_id]->mMovableParent;

		Matrix3d fixed_rotation = model.mFixedBodies[fbody_id]->mParentTransform.E.transpose();
		Vector3d fixed_position = model.mFixedBodies[fbody_id]->mParentTransform.r;

		Matrix3d parent_body_rotation = model.X_base[parent_id].E.transpose();
		Vector3d parent_body_position = model.X_base[parent_id].r;
		return parent_body_position + parent_body_rotation * (fixed_position + fixed_rotation * (point_body_coordinates));
	}

	Matrix3d body_rotation = model.X_base[body_id].E.transpose();
	Vector3d body_position = model.X_base[body_id].r;

	return body_position + body_rotation * point_body_coordinates;
}

ANY_RBDL_DLLAPI
Vector3d CalcBaseToBodyCoordinates (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_base_coordinates,
		bool update_kinematics) {
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	if (body_id >= model.fixed_body_discriminator) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		unsigned int parent_id = model.mFixedBodies[fbody_id]->mMovableParent;

		Matrix3d fixed_rotation = model.mFixedBodies[fbody_id]->mParentTransform.E;
		Vector3d fixed_position = model.mFixedBodies[fbody_id]->mParentTransform.r;

		Matrix3d parent_body_rotation = model.X_base[parent_id].E;
		Vector3d parent_body_position = model.X_base[parent_id].r;

		return fixed_rotation * ( - fixed_position - parent_body_rotation * (parent_body_position - point_base_coordinates));
	}

	Matrix3d body_rotation = model.X_base[body_id].E;
	Vector3d body_position = model.X_base[body_id].r;

	return body_rotation * (point_base_coordinates - body_position);
}

ANY_RBDL_DLLAPI
Matrix3d CalcBodyWorldOrientation (
		Model &model,
		const VectorNd &Q,
		const unsigned int body_id,
		bool update_kinematics) {
	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	if (body_id >= model.fixed_body_discriminator) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		model.mFixedBodies[fbody_id]->mBaseTransform = model.mFixedBodies[fbody_id]->mParentTransform * model.X_base[model.mFixedBodies[fbody_id]->mMovableParent];

		return model.mFixedBodies[fbody_id]->mBaseTransform.E;
	}

	return model.X_base[body_id].E;
}

// NOTE: Adapted for mimic
ANY_RBDL_DLLAPI
void CalcPointJacobian (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_position,
		MatrixNd &G,
		bool update_kinematics
	) {
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

	assert (G.rows() == 3 && G.cols() == model.qdot_size );

	// Make sure G is zero
	G.setZero();

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
	}

	unsigned int j = reference_body_id;

	// set of already processed mimic joints
	std::unordered_set<unsigned int> processed_mimics;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.

	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		// multiplier should be 1.0 when no mimic
		if (model.mJoints[j].mDoFCount == 3) {
			const Eigen::Matrix3d j33 = model.mJoints[j].mMimicMult * ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(3,0,3,3);
			RigidBodyDynamics::getMimicMatrix<3,3>(G.block<3,3>(0, q_index), j33, model, q_index, processed_mimics);
		} else {
			const Eigen::Vector3d j31 = model.mJoints[j].mMimicMult * point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block(3,0,3,1);
			RigidBodyDynamics::getMimicMatrix<3,1>(G.block<3,1>(0, q_index), j31, model, q_index, processed_mimics);
		}

		j = model.lambda[j];
	}
}

// NOTE: Adapted for mimic
ANY_RBDL_DLLAPI
void CalcBodySpatialJacobian (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		MatrixNd &G,
		bool update_kinematics
	) {
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	assert (G.rows() == 6 && G.cols() == model.qdot_size );

	// Make sure G is zero
	G.setZero();

	unsigned int reference_body_id = body_id;

	SpatialTransform base_to_body;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
		base_to_body = model.mFixedBodies[fbody_id]->mParentTransform * model.X_base[reference_body_id];
	} else {
		base_to_body = model.X_base[reference_body_id];
	}

	unsigned int j = reference_body_id;

	// set of already processed mimic joints
	std::unordered_set<unsigned int> processed_mimics;

	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		// multiplier should be 1.0 when no mimic
		if (model.mJoints[j].mDoFCount == 3) {
			const Eigen::Matrix<double,6,3> j63 = model.mJoints[j].mMimicMult * (base_to_body * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j];
			RigidBodyDynamics::getMimicMatrix<6,3>(G.block<6,3>(0,q_index), j63, model, q_index, processed_mimics);
		} else {
			const Eigen::Matrix<double,6,1> j61 = model.mJoints[j].mMimicMult * base_to_body.apply(model.X_base[j].inverse().apply(model.S[j]));
			RigidBodyDynamics::getMimicMatrix<6,1>(G.block<6,1>(0,q_index), j61, model, q_index, processed_mimics);
		}

		j = model.lambda[j];
	}
}

ANY_RBDL_DLLAPI
Vector3d CalcPointVelocity (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics
	) {
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;
	assert (model.IsBodyId(body_id));
	assert (model.q_size == Q.size());
	assert (model.qdot_size == QDot.size());

	// Reset the velocity of the root body
	model.v[0].setZero();

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, &QDot, NULL);
	}

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates (model, Q, reference_body_id, base_coords, false);

	}

	SpatialVector point_spatial_velocity = SpatialTransform (CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(), reference_point).apply(model.v[reference_body_id]);

	return Vector3d (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			);
}

ANY_RBDL_DLLAPI
Vector3d CalcPointAcceleration (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics,
		bool gravity
	)
{
	ANY_RBDL_LOG << "-------- " << __func__ << " --------" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0].setZero();

	if (update_kinematics || gravity)
		UpdateKinematics (model, Q, QDot, QDDot, gravity);

	ANY_RBDL_LOG << std::endl;

	unsigned int reference_body_id = body_id;
	Vector3d reference_point = point_position;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id]->mMovableParent;
		Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
		reference_point = CalcBaseToBodyCoordinates (model, Q, reference_body_id, base_coords, false);
	}

	SpatialTransform p_X_i (CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(), reference_point);

	SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
	Vector3d a_dash = Vector3d (p_v_i[0], p_v_i[1], p_v_i[2]).cross(Vector3d (p_v_i[3], p_v_i[4], p_v_i[5]));
	SpatialVector p_a_i = p_X_i.apply(model.a[reference_body_id]);

	return Vector3d (
			p_a_i[3] + a_dash[0],
			p_a_i[4] + a_dash[1],
			p_a_i[5] + a_dash[2]
			);
}

ANY_RBDL_DLLAPI
bool InverseKinematics (
		Model &model,
		const VectorNd &Qinit,
		const std::vector<unsigned int>& body_id,
		const std::vector<Vector3d>& body_point,
		const std::vector<Vector3d>& target_pos,
		VectorNd &Qres,
		double step_tol,
		double lambda,
		unsigned int max_iter
		) {

	assert (Qinit.size() == model.q_size);
	assert (body_id.size() == body_point.size());
	assert (body_id.size() == target_pos.size());

	MatrixNd J = MatrixNd::Zero(3 * body_id.size(), model.qdot_size);
	VectorNd e = VectorNd::Zero(3 * body_id.size());

	Qres = Qinit;

	for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++) {
		UpdateKinematicsCustom (model, &Qres, NULL, NULL);

		for (unsigned int k = 0; k < body_id.size(); k++) {
			MatrixNd G (MatrixNd::Zero(3, model.qdot_size));
			CalcPointJacobian (model, Qres, body_id[k], body_point[k], G, false);
			Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, body_id[k], body_point[k], false);
			ANY_RBDL_LOG << "current_pos = " << point_base.transpose() << std::endl;

			for (unsigned int i = 0; i < 3; i++) {
				for (unsigned int j = 0; j < model.qdot_size; j++) {
					unsigned int row = k * 3 + i;
					ANY_RBDL_LOG << "i = " << i << " j = " << j << " k = " << k << " row = " << row << " col = " << j << std::endl;
					J(row, j) = G (i,j);
				}

				e[k * 3 + i] = target_pos[k][i] - point_base[i];
			}

			ANY_RBDL_LOG << J << std::endl;

			// abort if we are getting "close"
			if (e.norm() < step_tol) {
				ANY_RBDL_LOG << "Reached target close enough after " << ik_iter << " steps" << std::endl;
				return true;
			}
		}

		ANY_RBDL_LOG << "J = " << J << std::endl;
		ANY_RBDL_LOG << "e = " << e.transpose() << std::endl;

		MatrixNd JJTe_lambda2_I = J * J.transpose() + lambda*lambda * MatrixNd::Identity(e.size(), e.size());

		VectorNd z (body_id.size() * 3);
#ifndef ANY_RBDL_USE_SIMPLE_MATH
		z = JJTe_lambda2_I.colPivHouseholderQr().solve (e);
#else
		bool solve_successful = LinSolveGaussElimPivot (JJTe_lambda2_I, e, z);
		assert (solve_successful);
#endif

		ANY_RBDL_LOG << "z = " << z << std::endl;

		VectorNd delta_theta = J.transpose() * z;
		ANY_RBDL_LOG << "change = " << delta_theta << std::endl;

		Qres = Qres + delta_theta;
		ANY_RBDL_LOG << "Qres = " << Qres.transpose() << std::endl;

		if (delta_theta.norm() < step_tol) {
			ANY_RBDL_LOG << "reached convergence after " << ik_iter << " steps" << std::endl;
			return true;
		}

		VectorNd test_1 (z.size());
		VectorNd test_res (z.size());

		test_1.setZero();

		for (unsigned int i = 0; i < z.size(); i++) {
			test_1[i] = 1.;

			VectorNd test_delta = J.transpose() * test_1;

			test_res[i] = test_delta.squaredNorm();

			test_1[i] = 0.;
		}

		ANY_RBDL_LOG << "test_res = " << test_res.transpose() << std::endl;
	}

	return false;
}

}
