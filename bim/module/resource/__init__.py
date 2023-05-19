# VertoolBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2020, 2021 Nainoor Patel
#
# This file is part of VertoolBIM Add-on.
#
# VertoolBIM Add-on is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# VertoolBIM Add-on is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with VertoolBIM Add-on.  If not, see <http://www.gnu.org/licenses/>.

import bpy
from . import ui, prop, operator

classes = (
    operator.DisableResourceEditingUI,
    operator.DisableEditingResource,
    operator.EnableEditingResource,
    operator.LoadResources,
    operator.AddResource,
    operator.AddResourceQuantity,
    operator.EditResource,
    operator.RemoveResource,
    operator.RemoveResourceQuantity,
    operator.LoadResourceProperties,
    operator.ExpandResource,
    operator.ContractResource,
    operator.AssignResource,
    operator.UnassignResource,
    operator.EnableEditingResourceTime,
    operator.EnableEditingResourceQuantity,
    operator.EnableEditingResourceBaseQuantity,
    operator.EnableEditingResourceCosts,
    operator.EnableEditingResourceCostValueFormula,
    operator.EnableEditingResourceCostValue,
    operator.EditResourceTime,
    operator.EditResourceQuantity,
    operator.EditResourceCostValue,
    operator.EditResourceCostValueFormula,
    operator.DisableEditingResourceTime,
    operator.DisableEditingResourceQuantity,
    operator.DisableEditingResourceCostValue,
    operator.CalculateResourceWork,
    operator.ImportResources,
    operator.EditProductivityData,
    prop.Resource,
    prop.BIMResourceProperties,
    prop.BIMResourceTreeProperties,
    prop.ISODuration,
    prop.BIMResourceProductivity,
    ui.BIM_PT_resources,
    ui.BIM_UL_resources,
)


def register():
    bpy.types.Scene.BIMResourceProperties = bpy.props.PointerProperty(type=prop.BIMResourceProperties)
    bpy.types.Scene.BIMResourceTreeProperties = bpy.props.PointerProperty(type=prop.BIMResourceTreeProperties)
    bpy.types.Scene.BIMResourceProductivity = bpy.props.PointerProperty(type=prop.BIMResourceProductivity)


def unregister():
    del bpy.types.Scene.BIMResourceProperties
    del bpy.types.Scene.BIMResourceTreeProperties
    del bpy.types.Scene.BIMResourceProductivity
