mechFizix is a light weight data oriented 3D physics engine.

Features
-Allocator
-Data containers
-math library
-Geometry library
-Dynamic octree
-Broadphase collision detection
-Narrowphase collision detection
-Constraint resolver
---and many ohters

how to get started 


/////////////////////////////////////////////////////////////////////////////////////////////////////


#inlcude<mechFizix/mechFizix.h>

int main()
{

	//Alwalys initialise the allocator first!!
	initialiseAllocator(nullptr, nullptr);
	
	mech::PhysicsWorld physicsWorld;
	
	//initialise octree 
	physicsWorld.initialiseOctree(mech::AABB(mech::Vec3(-300, -100, -300), mech::Vec3(300, 200, 300)), 4);

	//create parameters for a flat terrain
	mech::FlatTerrainParameters fParameters;
	fParameters.height = 0;
	fParameters.min = mech::Vec2(-1000, -1000);
	fParameters.max = mech::Vec2(1000, 1000);
	//initialise physicsWorld heightField using the parameters
	physicsWorld.initialiseHeightField(&fParameters, mech::GROUNDMATERIAL);
	
	//add convexHull to the physicsWorld
	mech::OBB obb = mech::OBB(mech::Vec3(), mech::Vec3(1));
	unsigned int convexHullID = physicsWorld.addConvexHull(obb.toConvexHull(), mech::ColliderMotionState::dynamic, mech::PLASTICMATERIAL, mech::Transform3D(mech::Vec3(-3, 58, 0)));
	
	//add sphere to the physicsWorld
	mech::Sphere sphere = mech::Sphere(mech::Vec3(), 1);
	unsigned int sphereID = physicsWorld.addSphere(sphere, mech::ColliderMotionState::dynamic, mech::RUBBERMATERIAL, mech::Transform3D(mech::Vec3(-2, 68, 0)));
	
	//add capsule to the physicsWorld
	mech::Capsule capsule = mech::Capsule(0.5, mech::Vec3(-0.5, 0, 0), mech::Vec3(0.5, 0, 0));
	unsigned int capsuleID = physicsWorld.addCapsule(capsule, mech::ColliderMotionState::dynamic, mech::RUBBERMATERIAL, mech::Transform3D(mech::Vec3(-2, 85, 0)));
	
	while(game.isRunning()){
		
		//update the physicsWorld world 
		//parameter deltaTime - time elapsed since the last frame
		physicsWorld.update(deltaTime);
		
		//fetch transform matrix to update body's position in the renderer
		mech::Mat4x4f convexHullTransform = physicsWorld.getRigidBody(convexHullID)->getTransformMatrix();
		mech::Mat4x4f sphereTransform = physicsWorld.getRigidBody(sphereID)->getTransformMatrix();
		mech::Mat4x4f capsuleTransform = physicsWorld.getRigidBody(capsuleID)->getTransformMatrix();
		 
		//add a force to a rigid body
		physicsWorld.getRigidBody(convexHullID)->addForce(mech::Vec3(10, 0, 0));
		
		//other game logic
	}
	
	return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
