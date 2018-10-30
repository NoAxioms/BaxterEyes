import subprocess
import os

output_directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/images/"
def ros_to_blender(coords):
    blender_coords =  [coords[1],coords[2],coords[0]]
    print("blender coords: " + str(blender_coords))
    return blender_coords
def ros_to_blender_quaternion(quat):
    # return [quat[3]] + ros_to_blender(quat[:3])
    blender_quat =  [quat[3],quat[1],quat[2],quat[0]]
    print("blender_quat: " + str(blender_quat))
    return blender_quat

def generate_image(target_ros, head_pose_ros, name = "dum.png"):
    target_blender = ros_to_blender(target_ros)
    face_location_blender = ros_to_blender(head_pose_ros[0])
    face_orientation_blender = ros_to_blender_quaternion(head_pose_ros[1])

    blender_command = """import bpy
import math
from mathutils import Euler




def cross(a, b):
    c = [a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0]]
    return c


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def magnitude(a):
    return (a[0]**2 + a[1]**2 + a[2]**2)**0.5

def normalize(a):
	mag = magnitude(a)
	return [i/mag for i in a]

def focus_eye(eye_name, target_global):
	#location is relative, use matrix_world.translation for global coordinates
    eye = bpy.data.objects[eye_name]
    #Reset axes
    eye.rotation_euler = Euler((0,0,0),'XYZ')
    iris = eye.children[0]
    #We want the iris to face the target location. Treat iris position as orientation of sphere
    eye_location_global = eye.matrix_world.translation
    iris_location_global = iris.matrix_world.translation
    iris_location_relative = [iris_location_global[x] - eye_location_global[x] for x in range(3)]
    
    # iris_location = (1,0,0)
    target_relative = [target_global[x] - eye_location_global[x] for x in range(3)]
    #Get an axis perpendicular to current and desired orientation vector, so we can rotate one to the other.
    rotation_axis = cross(iris_location_relative,target_relative)
    # a dot b = |a| |b| cos(angle)
    angle = math.acos(dot(target_relative, iris_location_relative) / \
        magnitude(target_relative) / magnitude(iris_location_relative))
    # Deselect all objects to avoid rotating them
    for obj in bpy.data.objects:
        obj.select = False
    eye.select = True
    bpy.ops.transform.rotate(value=angle, axis=rotation_axis)


# target_location = bpy.data.objects["Camera"].location
#Set face pose in blender to match actual pose
face = bpy.data.objects["Face"]
face.matrix_world.translation = %(face_location)s
face.rotation_mode = "QUATERNION"
face.rotation_quaternion = %(face_orientation)s

target_location = %(target_location)s
bpy.context.scene.update()
focus_eye("Eye_left", target_location)
focus_eye("Eye_right", target_location)
    """ % {
        "target_location":str(target_blender),
        "face_location":str(face_location_blender),
        "face_orientation":str(face_orientation_blender)
    }
    blender_argument_list = ["blender","eyes.blend","-b","--python-expr",blender_command,"-F","PNG","-o",output_directory + name,"-x","0","-f","1"]
    subprocess.call(blender_argument_list)
def generate_neutral_image(name = "neutral.png"):
    blender_command = """import bpy
import math
from mathutils import Euler

cam = bpy.data.cameras[0]
eyes = [bpy.data.objects["Eye_left"],bpy.data.objects["Eye_right"]]

if cam.type == "ORTHO":
    for eye in eyes:
        eye.rotation_euler = Euler((0,0,0),'XYZ')
else:
    def cross(a, b):
        c = [a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]]
        return c


    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


    def magnitude(a):
        return (a[0]**2 + a[1]**2 + a[2]**2)**0.5

    def normalize(a):
        mag = magnitude(a)
        return [i/mag for i in a]

    def focus_eye(eye_name, target_global):
        #location is relative, use matrix_world.translation for global coordinates
        eye = bpy.data.objects[eye_name]
        #Reset axes
        eye.rotation_euler = Euler((0,0,0),'XYZ')
        iris = eye.children[0]
        #We want the iris to face the target location. Treat iris position as orientation of sphere
        eye_location_global = eye.matrix_world.translation
        iris_location_global = iris.matrix_world.translation
        iris_location_relative = [iris_location_global[x] - eye_location_global[x] for x in range(3)]
        
        # iris_location = (1,0,0)
        target_relative = [target_global[x] - eye_location_global[x] for x in range(3)]
        #Get an axis perpendicular to current and desired orientation vector, so we can rotate one to the other.
        rotation_axis = cross(iris_location_relative,target_relative)
        # a dot b = |a| |b| cos(angle)
        angle = math.acos(dot(target_relative, iris_location_relative) / \
            magnitude(target_relative) / magnitude(iris_location_relative))
        # Deselect all objects to avoid rotating them
        for obj in bpy.data.objects:
            obj.select = False
        eye.select = True
        bpy.ops.transform.rotate(value=angle, axis=rotation_axis)


    target_location = bpy.data.objects["Camera"].matrix_world.translation
    bpy.context.scene.update()
    focus_eye("Eye_left", target_location)
    focus_eye("Eye_right", target_location)
    """
    blender_argument_list = ["blender","eyes.blend","-b","--python-expr",blender_command,"-F","PNG","-o",output_directory + name,"-x","0","-f","1"]
    subprocess.call(blender_argument_list)