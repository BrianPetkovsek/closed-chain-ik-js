#include <emscripten/bind.h>
#include "ccik/IKSolver.h"

using namespace emscripten;
using namespace ccik;

EMSCRIPTEN_BINDINGS( ccik_module ) {
	value_object<Vec3>( "Vec3" )
		.field( "x", &Vec3::x )
		.field( "y", &Vec3::y )
		.field( "z", &Vec3::z );

	enum_<JointMode>( "JointMode" )
		.value( "Rotation", JointMode::Rotation )
		.value( "Translation", JointMode::Translation );

	value_object<JointSpec>( "JointSpec" )
		.field( "axis", &JointSpec::axis )
		.field( "length", &JointSpec::length )
		.field( "mode", &JointSpec::mode )
		.field( "minLimit", &JointSpec::minLimit )
		.field( "maxLimit", &JointSpec::maxLimit )
		.field( "value", &JointSpec::value )
		.field( "name", &JointSpec::name );

	register_vector<JointSpec>( "JointSpecList" );
	register_vector<Vec3>( "Vec3List" );

	class_<Chain>( "Chain" )
		.constructor<>()
		.constructor<const std::vector<JointSpec> &, const Vec3 &>()
		.function( "setBasePosition", &Chain::setBasePosition )
		.function( "getBasePosition", &Chain::getBasePosition )
		.function( "setJoints", &Chain::setJoints )
		.function( "getJoints", &Chain::getJoints )
		.function( "getPositions", &Chain::getPositions )
		.function( "getEndEffector", &Chain::getEndEffector );

	class_<IKSolver>( "IKSolver" )
		.constructor<>()
		.constructor<const Chain &>()
		.function( "setChain", &IKSolver::setChain )
		.function( "getChain", &IKSolver::getChain )
		.function( "setTarget", &IKSolver::setTarget )
		.function( "getTarget", &IKSolver::getTarget )
		.function( "setDamping", &IKSolver::setDamping )
		.function( "setTolerance", &IKSolver::setTolerance )
		.function( "setStepScale", &IKSolver::setStepScale )
		.function( "solve", &IKSolver::solve )
		.function( "getPositions", &IKSolver::getPositions );

	constant( "CCIK_TOLERANCE", CCIK_TOLERANCE );
}
