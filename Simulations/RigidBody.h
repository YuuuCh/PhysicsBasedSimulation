#include "util/quaternion.h"

class Rigidbody {
public:
	Rigidbody(float m, Vec3 x, Quat q, Vec3 v, Vec3 w) :
		mass(m), Ibody(Mat4()), x(x), q(q), v(v), omega(w), f(Vec3()), torque(Vec3()) {
	}
	virtual void draw(DrawingUtilitiesClass* DUC) const = 0;
	double mass;
	Mat4 Ibody;
	Vec3 x;
	Quat q;
	Vec3 v;
	Vec3 omega;
	Vec3 f;
	Vec3 torque;
};

class Box : public Rigidbody {
public:
	Box(double m, Vec3 x, Quat q, Vec3 v, Vec3 w, Vec3 size) : size(size), Rigidbody(m,x,q,v,w) {
		double xs = size.x * size.x;
		double ys = size.y * size.y;
		double zs = size.z * size.z;
		Ibody = Mat4(ys + zs, 0, 0, 0, 0, xs + zs, 0, 0, 0, 0, xs + ys, 0, 0, 0, 0, 1) * mass / 12;
	}
	void draw(DrawingUtilitiesClass* DUC) const {
		Mat4 scale;
		Mat4 trans;
		scale.initScaling(size.x, size.y, size.z);
		trans.initTranslation(x.x, x.y, x.z);
		DUC->drawRigidBody(scale * q.getRotMat() * trans); // left handness
	}
private:
	const Vec3 size;
};

//class Sphere :public Rigidbody {
//public:
//	Sphere(float r, Vec3 p, Vec3 v, float m, Vec3 w, Quaternion<float> q)
//		:radius(r), Primitive(p, v, m, w, q) {}
//private:
//	float radius;
//};