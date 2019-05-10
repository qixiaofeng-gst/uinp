/*PUT js/object_utilities.js */
/*PUT js/shader_creator.js */
/*PUT js/input_engine.js */
/*PUT js/bone_engine.js */

/*
- DONE(1.5h) Mapping the screen coords to [-1, 1]
- 80% Shaders for rendering basic geometries
- 50% The physic mechanism for rigid body
  - method for checking point is inside polygon
  - collision
- The body editor
  - the ui engine, reuse bone_engine stuff
  - more basic shapes
    - line
    - soft strip
    - cloth
    - cycle
    - rectangle
    - solid strip (then rectangle with a free point at each end)
- Mapping the map coords to screen

- Make page auto reload after code change
- Use web socket to transfer script
*/

const canvas_size = {
  width: 1000,
  height: 900,
}
const prgm_render = create_shader_program('vs_render', 'fs_render')
const prgm_dot = create_shader_program('vs_dot', 'fs_dot')
const bff_quad = gl.createBuffer()
const draw_raw = (dots, shape, size) => {
  size = size || 2
  /*
    void gl.drawArrays(mode, first, count) count: number of indexes
    void gl.drawElements(mode, count, type, offset) count: number of elements
    
    void gl.vertexAttribPointer(index, size, type, normalized, stride, offset)
    
    gl.POINTS: Draws a single dot.
    gl.LINE_STRIP: Draws a straight line to the next vertex.
    gl.LINE_LOOP: Draws a straight line to the next vertex, and connects the last vertex back to the first.
    gl.LINES: Draws a line between a pair of vertices.
    gl.TRIANGLE_STRIP
    gl.TRIANGLE_FAN
    gl.TRIANGLES: Draws a triangle for a group of three vertices.
  */
  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(dots), gl.STATIC_DRAW)
  gl.vertexAttribPointer(0, size, gl.FLOAT, false, 0, 0)
  gl.enableVertexAttribArray(0)
  gl.drawArrays(shape, 0, dots.length / size)
}
const draw_lines = lines => draw_raw(lines, gl.LINES)
const draw_plane = plane => draw_raw(plane, gl.TRIANGLE_FAN, 4)
const points2dots = (points) => {
  const dots = []
  for (const [x, y, size] of points) {
    const offset = size * 0.5
    dots.push([
      x - offset, y - offset, -1, -1,
      x + offset, y - offset, 1, -1,
      x + offset, y + offset, 1, 1,
      x - offset, y + offset, -1, 1,
    ])
  }
  return dots
}

const init_scene = () => {
  gl.canvas.width = canvas_size.width
  gl.canvas.height = canvas_size.height
  
  const cvs_size = new Float32Array([
    canvas_size.width, canvas_size.height
  ])
  
  prgm_render.use()
  prgm_render.canvas_size = cvs_size
  prgm_dot.use()
  prgm_dot.canvas_size = cvs_size
  
  gl.clearColor(0, 0, 0, 0)
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
  gl.enable(gl.BLEND)
  
  gl.canvas.oncontextmenu = e => e.preventDefault()
}
init_scene()

const ie = InputEngine(gl.canvas)
const be = BoneEngine(gl.canvas, {show_stress: true})
const init_engines = () => {
  let dragging = false
  const
  end_drag = () => {
    if (dragging) {
      dragging.set_drag(false)
    }
    dragging = false
  },
  start_drag = () => {
    const { x, y } = ie.get_mouse()
    dragging = be.get_point_around(x, y)
    if (dragging) {
      dragging.set_drag(XY(x, y))
    }
  },
  do_drag = () => {
    if (dragging) {
      const { x, y } = ie.get_mouse()
      dragging.set_drag(XY(x, y))
    }
  }
  
  ie.onmousedown = start_drag
  ie.onmousemove = do_drag
  ie.onmouseup = end_drag
  
  ie.onkeydown = () => {
    if (ie.ctrl == ie.get_code()) {
      be.start_editing()
    }
  }
  ie.onclick = () => {
    if (ie.is_ctrl_down()) {
      const { x, y } = ie.get_mouse()
      be.create_point(x, y, ie.is_alt_down())
    }
  }
  ie.onkeyup = () => {
    if (ie.ctrl == ie.get_code()) {
      be.end_editing()
      be.detect_unities()
      console.log(be.get_unities().length)
    }
  }
  
  be.batch_add(deserialize(/*PUT objs/examples.js */))
}
init_engines()

const update = () => {
  be.update(16)
}
const render = () => {
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height)
  gl.bindFramebuffer(gl.FRAMEBUFFER, null)
  
  prgm_render.use()
  draw_lines(be.get_lines())
  
  prgm_dot.use()
  const dots = points2dots(be.get_points())
  for (const d of dots) {
    draw_plane(d)
  }
}

const main_loop = () => {
  update()
  render()
}
const looper = cb => {
  cb()
  requestAnimationFrame(delta_time => {
    looper(cb)
  })
}
looper(main_loop)
