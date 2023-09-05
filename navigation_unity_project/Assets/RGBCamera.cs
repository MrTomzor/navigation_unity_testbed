using UnityEngine;
using UnityEngine.Rendering;

namespace URP.Sensor
{
  [RequireComponent(typeof(Camera))]
  public class RGBCamera : MonoBehaviour
  {
    [SerializeField] private int _width  = 640;
    [SerializeField] private int _height = 480;
    [SerializeField] [Range(0,100)] int _quality = 50;
    [SerializeField] public float _scanRate = 30f;

    public uint width  { get => (uint)this._width; }
    public uint height { get => (uint)this._height; }
    public float scanRate { get => this._scanRate; }

    private Camera _camera;
    private Texture2D _texture;
    private Rect _rect;
    private RenderTexture rt;

    [HideInInspector] public byte[] data;

    public void Init()
    {
      this._camera  = GetComponent<Camera>();

      /* this._width = _camera.pixelWidth; */
      /* this._height = _camera.pixelHeight; */
      _camera.pixelRect = new Rect(0, 0, this._width, this._height);

      this._texture = new Texture2D(this._width, this._height, TextureFormat.RGB24, false);
      this._rect = new Rect(0, 0, this._width, this._height);
      this._texture.Apply();
      /* this._camera.targetTexture = new RenderTexture(this._width, this._height, 24); */

      /* rt = new RenderTexture(this._width, this._height, 16, RenderTextureFormat.DefaultHDR); */
      /* rt.enableRandomWrite = true; */
      /* rt.Create(); */
      /* RenderTexture.active = rt; */
      GL.Clear(true, true, Color.black);
      /* this._camera.targetTexture = rt; */


      _texture = new Texture2D(this._width, this._height);

      /* Camera.onPostRender += UpdateImage; */
      /* RenderPipelineManager.endCameraRendering += UpdateImage; */
    }

    public void ForceUpdateImage(Camera cam){
      /* RenderTexture screenTexture = new RenderTexture(this._width, this._height, 24); */
      /* cam.targetTexture = screenTexture; */
      /* RenderTexture.active = screenTexture; */
      /* cam.Render(); */
      /* Texture2D renderedTexture = new Texture2D(this._width, this._height); */
      /* renderedTexture.ReadPixels(new Rect(0, 0, this._width, this._height), 0, 0); */
      /* RenderTexture.active = null; */
      /* this.data = renderedTexture.EncodeToJPG(this._quality); */

      /* RenderTexture screenTexture = new RenderTexture(this._width, this._height, 24); */
      /* cam.pixelRect = new Rect(0, 0, this._width, this._height); */
      /* Debug.Log("CAMERA PIXELVALS: " + _camera.pixelWidth + ", " + _camera.pixelHeight); */

      var oldRec = cam.rect;
      cam.rect = new Rect(0f, 0f, 1f, 1f);

      var depth = 24;
      var format = RenderTextureFormat.Default;
      var readWrite = RenderTextureReadWrite.Default;

      var tempRt =
          RenderTexture.GetTemporary(this._width, this._height, 24);

      var prevActiveRt = RenderTexture.active;
      var prevCameraRt = cam.targetTexture;

      // render to offscreen texture (readonly from CPU side)
      RenderTexture.active = tempRt;
      cam.targetTexture = tempRt;

      cam.Render();

      _texture.ReadPixels(new Rect(0, 0, _texture.width, _texture.height), 0, 0);

      cam.targetTexture = prevCameraRt;
      cam.rect = oldRec;
      RenderTexture.active = prevActiveRt;
      RenderTexture.ReleaseTemporary(tempRt);

      this.data = _texture.EncodeToJPG(this._quality);

      /* byte[] byteArray = renderedTexture.EncodeToPNG(); */
      /* System.IO.File.WriteAllBytes(Application.dataPath + "/cameracapture.png", byteArray); */
    }

    public void UpdateImage(Camera _camera)
      /* private void UpdateImage(ScriptableRenderContext context, Camera _camera) */
    {
      if (this._texture != null && _camera == this._camera) {
        this._texture.ReadPixels(this._rect, 0, 0);
        /* this._texture.Apply(); */
        this.data = this._texture.EncodeToJPG(this._quality);
      }
    }
  }
}
