"""Shared helper for saving matplotlib FuncAnimation objects to disk."""

import os
import matplotlib.animation as animation


def save_animation(anim, out_path, fps=20):
    """Save anim as MP4 via ffmpeg if available, otherwise fall back to GIF."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    if animation.writers.is_available('ffmpeg'):
        anim.save(out_path, writer='ffmpeg', fps=fps, dpi=120)
        print(f"Saved video: {out_path}")
    else:
        gif_path = os.path.splitext(out_path)[0] + '.gif'
        anim.save(gif_path, writer='pillow', fps=fps)
        print(f"ffmpeg not found; saved GIF instead: {gif_path}")
