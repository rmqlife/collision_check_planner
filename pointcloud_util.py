import open3d as o3d
import numpy as np
import pymeshfix
from scipy.spatial import ConvexHull
import trimesh



class pointcloud:
    def __init__(self,pcd) -> None:
        self.pcd = pcd
        # self.compute_normals()

    def compute_normals(self):
        # Estimate normals using Open3D
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


    def clean_mesh(self,mesh):

        v,f = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        vclean, fclean = pymeshfix.clean_from_arrays(v, f)
        # create a new mesh
        new_mesh = o3d.geometry.TriangleMesh()
        new_mesh.vertices = o3d.utility.Vector3dVector(vclean)
        new_mesh.triangles = o3d.utility.Vector3iVector(fclean)
        self.mesh = new_mesh
        return new_mesh


    def create_mesh(self):

        # Compute nearest neighbor distances to estimate an average distance
        distances = self.pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        # Calculate a radius for the rolling ball algorithm
        radius = 1.5 * avg_dist
        # Create a triangular mesh using the Ball Pivoting algorithm from Open3D
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            self.pcd,
            o3d.utility.DoubleVector([radius, radius * 2]))
        self.mesh = mesh
        return mesh
    
    def get_pos(self):
        # get mesh position
        return self.pcd.get_center()
    

    def to_bounding_box(self, mesh):
        '''
        bullet have bounding box with getAABB() function
        '''


        # Convert the mesh to a point cloud
        points = mesh.sample_points_poisson_disk(300)
        points = np.asarray(points.points)
        hull = ConvexHull(points)
        vertices = points[hull.vertices]

        # Find minimum and maximum coordinates along each axis
        min_x = np.min(vertices[:, 0])
        max_x = np.max(vertices[:, 0])
        min_y = np.min(vertices[:, 1])
        max_y = np.max(vertices[:, 1])
        min_z = np.min(vertices[:, 2])
        max_z = np.max(vertices[:, 2])

        return min_x, min_y, min_z, max_x, max_y, max_z
    
    def get_mesh(self):
        return self.mesh

    def visualize(self, mesh):
        # Set color of the mesh to red
        mesh.paint_uniform_color([1.0, 0.0, 0.0])
        # Visualization using Open3D
        o3d.visualization.draw_geometries([mesh])


    def export(self, mesh, filename):
       # save as .obj file
        o3d.io.write_triangle_mesh(filename, mesh)

if __name__ == "__main__":
    mesh = trimesh.load('plydoc/mesh22.ply')

    print("Original Vertices:")
    print(mesh.vertices)
    original_faces_count = len(mesh.faces)
    target_faces_count = int(original_faces_count * 0.5)
    simplified_mesh = mesh.simplify_quadric_decimation(target_faces_count)

    # Export the cleaned mesh to an OBJ file
    simplified_mesh.export('plydoc/mesh22.obj')
    newmesh = trimesh.load('plydoc/mesh22.obj')
    print(newmesh.vertices)
    







    

