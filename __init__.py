bl_info = {
    "name": "blender-road",
    "author": "shkodykoshka",
    "version": (1, 0, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Sidebar > Create > Road",
    "description": "Makes AASHTO compliant road",
    "category": "Add Mesh",
}

import bpy
from bpy.app.handlers import persistent
from . import road_ops

class RoadConfig(bpy.types.PropertyGroup):
    road_width : bpy.props.FloatProperty(name="Road Width", description="Road Width in Meters", default=9.0, min=0.001, max = 3.402823e+38, precision=4)
    design_speed : bpy.props.IntProperty(name="Design Speed", description="Design speed in MPH", default=55, min=15, max=80)
    design_superelevation : bpy.props.FloatProperty(name="Design Super Elevation", description="Design super elevation in %", default=4, min=0, max=45, precision=4)
    # 10% is normal on highways where snow and ice do not exist
    # 8%  is 'logical' maximum on highways where snow and ice DO exist
    use_vector_radius : bpy.props.BoolProperty(name="Use Vector Radius", description="Uses distance between first and second selected vertex as radius instead of minimum radius for the selected speed and angle", default=False)
    update_mode : bpy.props.EnumProperty(items=[("MANUAL", "Manual", "", 0), ("AUTO", "Auto", "", 1)], name = "Update Mode", default = "MANUAL")
    segments : bpy.props.IntProperty(name="Segments", description="Segments", default=16, min=1, max=255)
    runoff_segments : bpy.props.IntProperty(name="Runoff Segments", description="Amount of segments to use for runoff sections", default=8, min=1, max=255)

# Update handler for road position or config change
@persistent
def road_change_handler(scene):
    if bpy.context.mode != "OBJECT":
        return
    for update in bpy.context.view_layer.depsgraph.updates:
        if update.is_updated_geometry:
            if bpy.context.scene.road_config.update_mode == "AUTO":
                road_ops.update_road()
                pass
            break

class RoadUI(bpy.types.Panel):
    bl_idname = "ROAD_PT_ops_ui"
    bl_label = "Road Configuration"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Road"
    
    def draw(self, context):
        config = context.scene.road_config
        # First section
        layout = self.layout
        col = layout.column()
        r1 = col.row(align=True)
        r1c1 = r1.column(align=True)
        # Buttons to draw and update road
        r1c1.operator("road_ops.draw_road", text="Draw Road")
        col.operator("road_ops.manual_update_road", text="Update Road")
        r2 = col.row(align=True)
        r2c1 = r2.column(align=True)
        r2c1.prop(config, "update_mode", text="Update Mode")

        # Second section
        col = layout.column()
        col.label(text="Configuration")
        # Rows for second section
        col.prop(config, "segments", text="Segments")
        col.prop(config, "runoff_segments", text="Runoff Segments")
        col.prop(config, "design_speed", text="Design Speed")
        col.prop(config, "road_width", text="Road Width")
        col.prop(config, "design_superelevation", text="Superelevation")
        col.label(text=" ")
        col.prop(config, "use_vector_radius", text="Use Vector Radius")

classes_to_register = [
    RoadConfig,
    RoadUI,
    road_ops.DrawRoad,
    road_ops.UpdateRoad
]

def register():
    for cls in classes_to_register:
        bpy.utils.register_class(cls)

    bpy.app.handlers.depsgraph_update_post.append(road_change_handler)
    bpy.types.Scene.road_config = bpy.props.PointerProperty(type=RoadConfig)

def unregister():
    del bpy.types.Scene.road_config
    bpy.app.handlers.depsgraph_update_post.remove(road_change_handler)
    for cls in classes_to_register:
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()