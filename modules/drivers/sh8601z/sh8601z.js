/*
 * Copyright (c) 2023  Pocuter Inc.
 * Copyright (c) 2016-2017  Moddable Tech, Inc.
 *
 *   This file is part of the Moddable SDK Runtime.
 * 
 *   The Moddable SDK Runtime is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 * 
 *   The Moddable SDK Runtime is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 * 
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with the Moddable SDK Runtime.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
	sh8601z SPI display controller
*/

import Bitmap from "commodetto/Bitmap";

export default class SH8601Z @ "xs_sh8601z_destructor" {
	constructor(dictionary) @ "xs_sh8601z";

	begin(x, y, width, height) @ "xs_sh8601z_begin";
	send(pixels, offset, count) @ "xs_sh8601z_send";
	end() @  "xs_sh8601z_end";

	adaptInvalid() @ "xs_sh8601z_adaptInvalid";
	continue() {
		// empty implementation overrides PixelOut.continue which throws
	}

	pixelsToBytes(count) {
		return (count * Bitmap.depth(this.pixelFormat)) >> 3;
	}

	get pixelFormat() @ "xs_sh8601z_get_pixelFormat";
	get width() @ "xs_sh8601z_get_width";
	get height() @ "xs_sh8601z_get_height";
	get async() {return true;}

	get clut() @ "xs_sh8601z_get_clut";
	set clut() @ "xs_sh8601z_set_clut";

	get c_dispatch() @ "xs_sh8601z_get_c_dispatch";
	
	// driver specific
	command(id, data) @ "xs_sh8601z_command";	
}

Object.freeze(SH8601Z.prototype);
