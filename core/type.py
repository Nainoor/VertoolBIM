# VertoolBIM Add-on - OpenBIM Blender Add-on
# Copyright (C) 2021 Nainoor Patel
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

import blenderbim.core.geometry


def assign_type(ifc, type_tool, element=None, type=None):
    ifc.run("type.assign_type", related_object=element, relating_type=type)
    obj = ifc.get_object(element)
    if type_tool.has_material_usage(element):
        pass # for now, representation regeneration handled by API listeners
    else:
        type_data = type_tool.get_object_data(ifc.get_object(type))
        if type_data:
            type_tool.change_object_data(obj, type_data, is_global=False)
    type_tool.disable_editing(obj)


def purge_unused_types(ifc, type):
    for element_type in type.get_model_types():
        if not type.get_type_occurrences(element_type):
            ifc.run("root.remove_product", product=element_type)
            obj = ifc.get_object(element_type)
            if obj:
                ifc.unlink(obj=obj)
                type.remove_object(obj)
