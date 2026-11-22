"""Read the checkout's server .env.local without executing shell code."""
import os
from pathlib import Path
import shlex


def server_settings():
    explicit = os.getenv('BOX_SERVER_ENV_FILE')
    candidates = [Path(explicit).expanduser()] if explicit else [
        root / 'my-next-app' / '.env.local' for root in Path(__file__).resolve().parents]
    values = {}
    for path in candidates:
        if not path.is_file():
            continue
        for line in path.read_text().splitlines():
            name, separator, value = line.strip().removeprefix('export ').partition('=')
            if not separator or name not in ('SUPABASE_URL','NEXT_PUBLIC_SUPABASE_URL','SUPABASE_SERVICE_KEY'):
                continue
            parts = shlex.split(value, comments=True, posix=True)
            if len(parts) == 1:
                values[name] = parts[0]
        break
    url = os.getenv('SUPABASE_URL') or os.getenv('NEXT_PUBLIC_SUPABASE_URL') or values.get('SUPABASE_URL') or values.get('NEXT_PUBLIC_SUPABASE_URL', '')
    key = os.getenv('SUPABASE_SERVICE_KEY') or values.get('SUPABASE_SERVICE_KEY', '')
    return url, key
