#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include <ostream>
#include <bits/stdc++.h>

using namespace Eigen;
using namespace std;

/**
 * A class for dealin with the computation of Bezier curves and their properties
 **/
class Bezier
{

public:
  /**
 * An iterative implementation of the De Casteljau algorithm for computing a Bezier curve
 * 
 * @param V  the vertices of the control polygon
 * @param t   an input parameter in [0, 1]
 * 
 * @return    the point B(t), obtaining by evaluating the curve for the value 't' 
 **/
  MatrixXd de_casteljau(const MatrixXd &V, double t)
  {
    int nV = V.rows();         // number of points in the control polygon
    int degree = V.rows() - 1; // degree of the curve
    //completed (ex 1)
    int n = nV-1;
    MatrixXd VCrawl = V;

    for(int r=1;r<=n;r++){
    	for(int i=0;i<=n-r;i++){
    		VCrawl(i,0) = (1-t)*VCrawl(i,0) + t*VCrawl(i+1,0);
    		VCrawl(i,1) = (1-t)*VCrawl(i,1) + t*VCrawl(i+1,1);
    		VCrawl(i,2) = (1-t)*VCrawl(i,2) + t*VCrawl(i+1,2);
    	}
    }

    MatrixXd ans(1,3);
    ans(0,0) = VCrawl(0,0);
    ans(0,1) = VCrawl(0,1);
    ans(0,2) = VCrawl(0,2);

    return ans;
  }

  /**
	 * Plot the curve, for t=0, dt, 2*dt, ..., 1, with a given 'resolution' <br>
   * where dt=1/(resolution-1)
   * 
   * @param resolution  number of points to be evaluated on the curve
	 */
  MatrixXd plot_curve(const MatrixXd &V, int resolution)
  {
    double dt = 1. / (resolution - 1);

    MatrixXd bezierPlot(resolution, 3);
    MatrixXd crawl;

    for(int i=0;i<resolution;i++){
    	crawl = de_casteljau(V, 1.0*i*dt);
    	bezierPlot(i,0) = crawl(0,0);
    	bezierPlot(i,1) = crawl(0,1);
    	bezierPlot(i,2) = crawl(0,2);
    }

    return bezierPlot;
  }

  /**
	 * Perform the subdivision (once) of the Bezier curve (with parameter t) <br>
	 * Return two Bezier curves (with 'n' control points each)
	 */
  vector<MatrixXd> subdivide(const MatrixXd &V, double t)
  {
  	//completed (ex 1)
    vector<MatrixXd> curves{}; // the result: store the 2 curves obtained after subdivision
    int nV = V.rows();         
    MatrixXd left(nV,3), right(nV,3);
    int n = nV-1, contLeft = 0, contRight = n;
    MatrixXd VCrawl = V;

    left(contLeft,0) = VCrawl(0,0);
    left(contLeft,1) = VCrawl(0,1);
    left(contLeft,2) = VCrawl(0,2);
    right(contRight,0) = VCrawl(n,0);
    right(contRight,1) = VCrawl(n,1);
    right(contRight,2) = VCrawl(n,2);
    contLeft++;
    contRight--;

    for(int r=1;r<=n;r++){
    	for(int i=0;i<=n-r;i++){
    		VCrawl(i,0) = (1-t)*VCrawl(i,0) + t*VCrawl(i+1,0);
    		VCrawl(i,1) = (1-t)*VCrawl(i,1) + t*VCrawl(i+1,1);
    		VCrawl(i,2) = (1-t)*VCrawl(i,2) + t*VCrawl(i+1,2);
    	}

    	left(contLeft,0) = VCrawl(0,0);
    	left(contLeft,1) = VCrawl(0,1);
    	left(contLeft,2) = VCrawl(0,2);
    	right(contRight,0) = VCrawl(n-r,0);
    	right(contRight,1) = VCrawl(n-r,1);
    	right(contRight,2) = VCrawl(n-r,2);
    	contLeft++;
    	contRight--;
    }

    curves.push_back(left);
    curves.push_back(right);

    return curves;
  }

  /**
	 * Plot the curve using recursive subdivision <br>
   * 
   * @param levels  number of levels of subdivisions
   * @return  a polyline representing the curve to be rendered: this is obtained by concantenation of
   * the control points of all subdivided curves
	 */
  MatrixXd subdivision_plot(const MatrixXd &V, int levels)
  {
    std::cout << "computing recursive subdivision " << std::endl;
    //list<MatrixXd> curves{};
    //completed

    int nodeNumber = 0, level = 12, nV = V.rows() - 1, MAXN = (1 << level);
    MatrixXd V1 = V;
    vector<MatrixXd> nodes;
    queue<int> q;
    bool visit[MAXN+1]; //visit matrix for the in-order traversal
    bool leaf[MAXN+1];  //indicate if the node is a leaf or not
    memset(visit, false, sizeof(visit));
    memset(leaf, false, sizeof(leaf));

    //Constructiong the binary tree representing the subdivisions (with a bfs)
    q.push(nodeNumber+1);
    nodes.push_back(V1);
    nodeNumber++;

    while(true){
    	MatrixXd cur = nodes[q.front() - 1];
    	q.pop();

    	q.push(nodeNumber+1);
    	q.push(nodeNumber+2);

    	if((nodeNumber+1) >= (1 << (level - 1)) && (nodeNumber+1) <= ((1 << level) - 1)) leaf[nodeNumber+1] = true;
    	if((nodeNumber+2) >= (1 << (level - 1)) && (nodeNumber+2) <= ((1 << level) - 1)) leaf[nodeNumber+2] = true;

    	vector<MatrixXd> div = subdivide(cur, 0.5);

    	nodes.push_back(div[0]);
    	nodes.push_back(div[1]);

    	nodeNumber += 2;

    	if(nodeNumber == (1 << level) - 1) break;
    }

    //Doing an in-order traversal of the tree with a stack (implicit recursion)
    nodeNumber = 0;
    stack<int> st;
    vector<int> finalOrder;

    st.push(nodeNumber+1);
    nodeNumber++;

    while(!st.empty()){
    	int curNumber = st.top();
    	st.pop();

    	if(visit[curNumber]) finalOrder.push_back(curNumber);
    	else{
    		if(!leaf[curNumber]) st.push(2*curNumber+1);
    		st.push(curNumber);
    		if(!leaf[curNumber]) st.push(2*curNumber);
    		visit[curNumber] = true;
    	}
    }

    //Must remember to include the two extremes vertices
    MatrixXd ans((int)finalOrder.size() + 1, 3);
    ans(0,0) = V1(0,0);
    ans(0,1) = V1(0,1);
    ans(0,2) = V1(0,2);

    for(int i=0;i < (int)finalOrder.size(); i++){
    	MatrixXd cur = de_casteljau(nodes[finalOrder[i] - 1], 0.5);
    	ans(i+1,0) = cur(0,0);
    	ans(i+1,1) = cur(0,1);
    	ans(i+1,2) = cur(0,2);
    }

    ans((int)finalOrder.size(),0) = V1(nV,0);
    ans((int)finalOrder.size(),1) = V1(nV,1);
    ans((int)finalOrder.size(),2) = V1(nV,2);

    return ans;
  }

  //function to calculate C(n, k)
  int C(int n, int k) {
    double res = 1;
    for (int i = 1; i <= k; ++i)
        res = res * (n - k + i) / i;
    return (int)(res + 0.01);
  }

  //function to calculate Bernstein polynomials
  float B(int i, int n, float t){
  	return 1.0*C(n,i)*pow(t,i)*pow(1.0-t,n-i);
  }

  /**
 * Compute the tangent of a given curve c(t) for a given parameter t0
 * 
 * @param V  the vertices of the control polygon
 * @param t0   an input parameter in [0, 1]
 * 
 * @return    the tangent at c(t0)
 **/
  MatrixXd compute_tangent(const MatrixXd &V, double t0)
  {
    int n = V.rows() - 1; // number of points in the control polygon
    //completed
    float tang0 = 0.0, tang1 = 0.0, tang2 = 0.0;

    for(int i=0;i<=n-1;i++) tang0 += (V(i+1,0) - V(i,0))*B(i,n-1,t0);
    tang0 *= 1.0*n;
    for(int i=0;i<=n-1;i++) tang1 += (V(i+1,1) - V(i,1))*B(i,n-1,t0);
    tang1 *= 1.0*n;
    for(int i=0;i<=n-1;i++) tang2 += (V(i+1,2) - V(i,2))*B(i,n-1,t0);
    tang2 *= 1.0*n;

    Vector3d tang(tang0, tang1, tang2);
    tang.normalize();

    MatrixXd ans(1,3);
    ans(0,0) = tang(0);
    ans(0,1) = tang(1);
    ans(0,2) = tang(2);

    return ans;
  }

  /**
 * Compute the normal vector of a given curve c(t) for a given parameter t0
 * 
 * @param V  the vertices of the control polygon
 * @param t0   an input parameter in [0, 1]
 * 
 * @return    the normal at c(t0)
 **/
  MatrixXd compute_normal(const MatrixXd &V, double t0)
  {
    int n = V.rows() - 1; // number of points in the control polygon
    //completed
    MatrixXd tangent = compute_tangent(V, t0);
    Vector3d v(V(1,0)- V(0,0), V(1,1) - V(0,1), V(1,2) - V(0,2));
    Vector3d tang(tangent(0,0), tangent(0,1), tangent(0,2));
    Vector3d normal = v.cross(tang);
    normal = normal.cross(tang);
    normal.normalize();

    MatrixXd norm(1,3);
    norm(0,0) = normal(0);
    norm(0,1) = normal(1);
    norm(0,2) = normal(2);
    return norm;
  }

  //Rotate the set of points V by an angle theta around an axis u using quaternions
  void transform(MatrixXd &V, RowVector3d u, double theta) {
    Quaterniond rot = Quaterniond(cos(theta/2.0), u(0,0)*sin(theta/2.0), u(0,1)*sin(theta/2.0), u(0,2)*sin(theta/2.0));
    Quaterniond rot_conj = Quaterniond(cos(theta/2.0), -u(0,0)*sin(theta/2.0), -u(0,1)*sin(theta/2.0), -u(0,2)*sin(theta/2.0));
    int n = V.rows();

    for(int i=0;i<n;i++){
      Quaterniond q = Quaterniond(0.0, V(i,0), V(i,1), V(i,2));
      Quaterniond rotated = rot*q;
      rotated = rotated*rot_conj;
      V(i,0) = rotated.x();
      V(i,1) = rotated.y();
      V(i,2) = rotated.z();
    }
  }

  /**
 * Compute a loop of points around a curve c(t) for a given parameter t0
 * The points belongs on a circle lying the hyperplane passing through c(t0) and orthogonal to tangent(t0)
 * 
 * @param V  the vertices of the control polygon
 * @param t0   an input parameter in [0, 1]
 * 
 * @return    a loop of vertices on the hyperplane passing through c(t0) and orthogonal to tangent(t0)
 **/
  MatrixXd compute_loop_of_vertices(const MatrixXd &V, double t0, int k, double radius)
  {
    int n = V.rows(); // number of points in the control polygon
    //completed
    MatrixXd P = de_casteljau(V, t0);
    MatrixXd tang = compute_tangent(V, t0);
    MatrixXd norm = compute_normal(V, t0);

    //It's necessary to translate the point to the origin in order to rotate with quaternions
    MatrixXd P0(1,3);
    P0(0,0) = radius*norm(0,0);
    P0(0,1) = radius*norm(0,1);
    P0(0,2) = radius*norm(0,2);

    RowVector3d tang_vector(tang(0,0), tang(0,1), tang(0,2));
    double PI = acos(-1.0);

    //Now we retranslate to coordinates with origin P to get the points of the circle
    MatrixXd loop(k,3);
    loop(0,0) = P0(0,0) + P(0,0);
    loop(0,1) = P0(0,1) + P(0,1);
    loop(0,2) = P0(0,2) + P(0,2);

    for(int i=1;i<k;i++){
    	transform(P0, tang_vector, (2*PI)/k);
    	loop(i,0) = P0(0,0) + P(0,0);
    	loop(i,1) = P0(0,1) + P(0,1);
    	loop(i,2) = P0(0,2) + P(0,2);
    }

    return loop;
  }

};
