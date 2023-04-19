#include "util/quaternion.h"

class Rigidbody {
public:
	Rigidbody(float m, Vec3 x, Quat q, Vec3 v, Vec3 w, bool is_fixed = false) :
		mass(m), Ibody(Mat4()), x(x), q(q), v(v), omega(w), f(Vec3()), torque(Vec3()), is_fixed(is_fixed) {
	}
	virtual void draw(DrawingUtilitiesClass* DUC) const = 0;
	virtual Mat4 getWorldMat() const = 0;
	virtual void applyImpulse(double J, Vec3 n, Vec3 loc) = 0;
	virtual void applyForce(Vec3 loc, Vec3 force) = 0;
	virtual Vec3 velOnBody(Vec3 loc) const = 0;
	double mass;
	Mat4 Ibody;
	Vec3 x;
	Quat q;
	Vec3 v;
	Vec3 omega;
	Vec3 f;
	Vec3 torque;
	bool is_fixed;
};

class Box : public Rigidbody {
public:
	Box(double m, Vec3 x, Quat q, Vec3 v, Vec3 w, Vec3 size, bool is_fixed = false) : size(size), Rigidbody(m,x,q,v,w,is_fixed) {
		double xs = size.x * size.x;
		double ys = size.y * size.y;
		double zs = size.z * size.z;
		Ibody = Mat4(ys + zs, 0, 0, 0, 0, xs + zs, 0, 0, 0, 0, xs + ys, 0, 0, 0, 0, 1) * mass / 12;
	}
	void draw(DrawingUtilitiesClass* DUC) const override {
		DUC->drawRigidBody(getWorldMat());
	}
	Mat4 getWorldMat() const override {
		Mat4 scale;
		Mat4 trans;
		scale.initScaling(size.x, size.y, size.z);
		trans.initTranslation(x.x, x.y, x.z);
		return scale * q.getRotMat() * trans; // left handness
	}
	void applyImpulse(double J, Vec3 n, Vec3 loc) override {
		if (is_fixed)return;
		v += J * n / mass;
		omega += cross(loc - x, J * n);
	}
	void applyForce(Vec3 loc, Vec3 force) override {
		if (is_fixed)return;
		Vec3 r = loc - x;
		//if (std::abs(r.x) > size.x / 2 || std::abs(r.y) > size.y / 2 || std::abs(r.z) > size.z / 2)
		//	return;
		f += force;
		torque += cross(r, force);
	}
	Vec3 velOnBody(Vec3 loc) const override {
		return v + cross(omega, loc - x);
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