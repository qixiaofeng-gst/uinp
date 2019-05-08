/*PUT js/shader_creator.js */
/*PUT js/bone_engine.js */

/*
- DONE(1.5h) Mapping the screen coords to [-1, 1]
- 50% Shaders for rendering basic geometries
- 30% The physic mechanism for rigid body
- The body editor
- Mapping the map coords to screen

- Make page auto reload after code change
- Use web socket to transfer script
*/

const canvas_size = {
  width: 1000,
  height: 900,
}
const prgm_render = create_shader_program('vs_render', 'fs_render')
const bff_quad = gl.createBuffer()
const draw_lines = lines => {
  /*
    void gl.drawArrays(mode, first, count) count: number of indexes
    void gl.drawElements(mode, count, type, offset) count: number of elements
    
    gl.POINTS: Draws a single dot.
    gl.LINE_STRIP: Draws a straight line to the next vertex.
    gl.LINE_LOOP: Draws a straight line to the next vertex, and connects the last vertex back to the first.
    gl.LINES: Draws a line between a pair of vertices.
    gl.TRIANGLE_STRIP
    gl.TRIANGLE_FAN
    gl.TRIANGLES: Draws a triangle for a group of three vertices.
  */
  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lines), gl.STATIC_DRAW)
  gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0)
  gl.enableVertexAttribArray(0)
  gl.drawArrays(gl.LINES, 0, lines.length / 2)
}

const init_scene = () => {
  gl.canvas.width = canvas_size.width
  gl.canvas.height = canvas_size.height
  
  prgm_render.use()
  prgm_render.canvas_size = new Float32Array([
    canvas_size.width, canvas_size.height
  ])
  
  gl.clearColor(0, 0, 0, 0)
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

  gl.canvas.onclick = ({ offsetX, offsetY }) => {
    console.log(offsetX, offsetY, '<<<<<<<')
  }
}
init_scene()

const jsv = BoneEngine(gl.canvas, {show_stress: true})
const init_engine = () => {
  const
  p1  = jsv.add_point(90, 40, false),
  p2  = jsv.add_point(160, 40, false),
  p3  = jsv.add_point(90, 110, false),
  p4  = jsv.add_point(160, 110, false),
  p5  = jsv.add_point(90, 180, false),
  p6  = jsv.add_point(160, 180, false),
  p7  = jsv.add_point(90, 250, false),
  p8  = jsv.add_point(160, 250, false),
  p9  = jsv.add_point(90, 320, true),
  p10 = jsv.add_point(160, 320, true),
  p11 = jsv.add_point(300, 40, false),
  p12 = jsv.add_point(365, 198, false),
  p13 = jsv.add_point(345, 218, false),
  p14 = jsv.add_point(385, 218, false)

  jsv.add_constraint(p1, p2)
  jsv.add_constraint(p3, p4)
  jsv.add_constraint(p5, p6)
  jsv.add_constraint(p7, p8)
  jsv.add_constraint(p1, p3)
  jsv.add_constraint(p3, p5)
  jsv.add_constraint(p5, p7)
  jsv.add_constraint(p7, p9)
  jsv.add_constraint(p2, p4)
  jsv.add_constraint(p4, p6)
  jsv.add_constraint(p6, p8)
  jsv.add_constraint(p8, p10)
  jsv.add_constraint(p1, p4)
  jsv.add_constraint(p2, p3)
  jsv.add_constraint(p3, p6)
  jsv.add_constraint(p4, p5)
  jsv.add_constraint(p5, p8)
  jsv.add_constraint(p6, p7)
  jsv.add_constraint(p7, p10)
  jsv.add_constraint(p8, p9)
  jsv.add_constraint(p2, p11)
  jsv.add_constraint(p4, p11)
  jsv.add_constraint(p11, p12)
  jsv.add_constraint(p12, p13)
  jsv.add_constraint(p12, p14)
  jsv.add_constraint(p13, p14)
  
  jsv.add_shape(Rectangle(500, 70, 70, 70))

  const square = Rectangle(630, 70, 50, 50)
  square.fix(1)
  jsv.add_shape(square)
}
init_engine()

const update = () => {
  jsv.update(16)
}
const render = () => {
  prgm_render.use()
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height)
  gl.enable(gl.BLEND)
  gl.bindFramebuffer(gl.FRAMEBUFFER, null)
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

  draw_lines(jsv.get_lines())
  gl.disable(gl.BLEND)
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
