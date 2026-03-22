/**
 * CameraView — Stream MJPEG desde /video_feed del backend Pi5
 */
export default function CameraView() {
  const src = `http://${window.location.hostname}:8000/video_feed`

  return (
    <div className="flex flex-col items-center justify-center h-full bg-black gap-3 p-2">
      <img
        src={src}
        alt="Cámara Kinect"
        className="w-full max-h-[75vh] object-contain rounded-xl border border-trolley-border"
        onError={e => { e.target.style.opacity = '0.3' }}
      />
      <p className="text-xs text-slate-500">Kinect RGB — 15 fps</p>
    </div>
  )
}
