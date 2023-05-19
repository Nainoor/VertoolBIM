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


def resize_to_storey(misc, obj=None, total_storeys=None):
    storey = misc.get_object_storey(obj)
    if not storey:
        return
    height = misc.get_storey_height_in_si(storey, total_storeys)
    if not height:
        return
    misc.set_object_origin_to_bottom(obj)
    misc.move_object_to_elevation(obj, misc.get_storey_elevation_in_si(storey))
    misc.scale_object_to_height(obj, height)
    misc.mark_object_as_edited(obj)


def split_along_edge(misc, cutter=None, objs=None):
    new_objs = misc.split_objects_with_cutter(objs, cutter)
    for obj in new_objs:
        misc.run_root_copy_class(obj=obj)
    for obj in objs:
        misc.mark_object_as_edited(obj)
