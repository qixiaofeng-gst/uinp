/*PUT js/shader_creator.js */

/*
- DONE(1.5h) Mapping the screen coords to [-1, 1]
- Shaders for rendering basic geometries
- The physic mechanism for rigid body
- The body editor
- Mapping the map coords to screen

- Make page auto reload after code change
- Use web socket to transfer script
*/

const prgm_render = create_shader_program('vs_render', 'fs_render')
const image = document.getElementById('texture')
const bff_quad = gl.createBuffer()
const arr_vtx_quad = [
  2, 2,
  250, 2,
  250, 170,
  2, 170,
]
const draw_quad = () => {
  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0)
  gl.drawArrays(gl.TRIANGLE_FAN, 0, 4)
  //gl.drawArrays(gl.LINE_LOOP, 0, 4)
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
}

const init_scene = () => {
  prgm_render.use()
  gl.enableVertexAttribArray(0)
  gl.bindBuffer(gl.ARRAY_BUFFER, bff_quad)
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(arr_vtx_quad), gl.STATIC_DRAW)
  prgm_render.canvas_size = new Float32Array([
    500, 340
  ])
  
  gl.clearColor(0.5, 0.5, 0, 0)
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
    
  gl.canvas.onclick = ({ offsetX, offsetY }) => {
    console.log(offsetX, offsetY, '<<<<<<<')
  }
}
init_scene()

const update = () => {
  //TODO
}
const render = () => {
  prgm_render.use()
  gl.viewport(0, 0, gl.canvas.width, gl.canvas.height)
  gl.enable(gl.BLEND)
  gl.bindFramebuffer(gl.FRAMEBUFFER, null)
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

  draw_quad()
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
