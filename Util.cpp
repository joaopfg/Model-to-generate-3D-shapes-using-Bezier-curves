#include <igl/opengl/glfw/Viewer.h>
#include <ostream>

using namespace Eigen;
using namespace std;

/**
 * A class providing tools for drawing 3d curves
 * */
class Util
{
public:
  /*
Initialize the viewer
**/
  Util(igl::opengl::glfw::Viewer &v)
  {
    viewer = &v;
    viewer->append_mesh();
    viewer->append_mesh();
    viewer->append_mesh();
    //viewer->append_mesh();
    red = new RowVector3d(0.9, 0.1, 0.1);
    green = new RowVector3d(0.1, 0.9, 0.1);
    yellow = new RowVector3d(1.0, 1.0, 0.1);
  }

  /*
draw tangent vectors
**/
  void draw_tangents(const MatrixXd &V, const MatrixXd &dX)
  {
    std::cout << "Drawing tangents " << std::endl;
    RowVector3d c = *green;
    //viewer->append_mesh();
    for (int i = 0; i < V.rows(); i++)
    {
      std::cout << i << " tangent at " << V.row(i) << ": " << dX << std::endl;
      viewer->data(2).add_edges(
          V.row(i),
          V.row(i) + dX.row(i),
          c);
    }
  }

  /*
draw tangent vectors
**/
  void draw_normals(const MatrixXd &V, const MatrixXd &dX)
  {
    std::cout << "Drawing tangents " << std::endl;
    RowVector3d c = *yellow;
    //viewer->append_mesh();
    for (int i = 0; i < V.rows(); i++)
    {
      std::cout << i << " tangent at " << V.row(i) << ": " << dX << std::endl;
      viewer->data(2).add_edges(
          V.row(i),
          V.row(i) + dX.row(i),
          c);
    }
  }

  /*
draw a curve in 3D
**/
  void draw_curve(const MatrixXd &V)
  {
    //viewer->append_mesh();
    for (unsigned i = 0; i < V.rows() - 1; ++i)
      viewer->data(1).add_edges(
          V.row(i),
          V.row(i + 1),
          Eigen::RowVector3d(1, 1, 1));
  }

  /*
draw a curve in 3D
**/
  void draw_colored_curve(const MatrixXd &V, RowVector3d &color)
  {
    //viewer->append_mesh();
    for (unsigned i = 0; i < V.rows() - 1; ++i)
      viewer->data(1).add_edges(
          V.row(i),
          V.row(i + 1),
          color);
  }

  /*
draw a curve in 3D
**/
  void draw_close_curve(const MatrixXd &V)
  {
    //viewer->append_mesh();
    for (unsigned i = 0; i < V.rows() - 1; ++i)
    {
      viewer->data(1).add_edges(
          V.row(i),
          V.row(i + 1),
          Eigen::RowVector3d(0.1, 0.8, 0.1));
    }
    viewer->data(1).add_edges(V.row(V.rows() - 1), V.row(0), Eigen::RowVector3d(0.1, 0.8, 0.1));
  }

  /*
draw a curve in 3D
**/
  void draw_cylinder_grid(MatrixXd points[], int h)
  {
    for (int i = 0; i < h; i++)
    {
      draw_close_curve(points[i]);
    }

    int w=points[0].rows();
    for (unsigned i = 0; i < w; ++i)
    {
      for(int j=0;j<h-1;j++)
      viewer->data(1).add_edges(points[j].row(i), points[j+1].row(i), Eigen::RowVector3d(0.1, 0.8, 0.1));
    }
  }

  /*
draw a set of points in 3D
**/
  void draw_control_polygon(const MatrixXd &V)
  {
    RowVector3d r = *red;
    //viewer->append_mesh();
    for (unsigned i = 0; i < V.rows() - 1; ++i)
      viewer->data(0).add_edges(
          V.row(i),
          V.row(i + 1),
          r);

    viewer->data(0).add_points(V, r);
  }

  /**
* draw a set of points in 3D
**/
  void draw_edges(const MatrixXd &V)
  {
    for (unsigned i = 0; i < V.rows() - 1; ++i)
      viewer->data(0).add_edges(
          V.row(i),
          V.row(i + 1),
          Eigen::RowVector3d(1, 1, 1));
  }

  /**
* translate of the points of a vector 'a'
**/
  MatrixXd translate_points(const MatrixXd &V, MatrixXd a)
  {
    MatrixXd result = V; // copy the input points
    for (int i = 0; i < V.rows(); ++i)
    {
      MatrixXd v = V.row(i);
      result.row(i) = (v + a);
    }
    return result;
  }

  const Eigen::RowVector3d *red, *green, *yellow;

private:

  igl::opengl::glfw::Viewer *viewer; // a 4x4 matrix representing a linear transformation
};
