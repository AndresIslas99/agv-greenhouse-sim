# Going-public checklist — agv-greenhouse-sim

Steps to open this repository. The working tree has been scrubbed of the
committed credentials, usernames, and LAN IPs (see the PR that added this
file), but **the secrets remain in git history** and a file edit does not
remove them from earlier commits. Do the history rewrite **before** making
the repo public.

## 1. Rewrite history to purge the leaked secrets

```bash
# From a FRESH mirror clone (never on your only copy — this rewrites everything)
git clone --mirror https://github.com/AndresIslas99/agv-greenhouse-sim.git
cd agv-greenhouse-sim.git
pip install git-filter-repo

# Put each leaked string on its own line, "literal==>replacement".
# The critical one is the Jetson SSH/VNC password; also the site LAN IPs,
# the operator usernames, and the personal home path.
cat > /tmp/replacements.txt <<'EOF'
<JETSON_PASSWORD>==>REDACTED
192.168.15.241==>JETSON-LAN-IP
192.168.15.79==>SIM-HOST-LAN-IP
/home/andres==>/home/USER
orza==>jetson-user
EOF

git filter-repo --replace-text /tmp/replacements.txt --force

# Verify nothing survives (should print nothing):
git grep -nI '192\.168\.15\.\|sshpass' $(git rev-list --all)

# Only if clean — this is irreversible:
git push --force --mirror https://github.com/AndresIslas99/agv-greenhouse-sim.git
```

Replace `<JETSON_PASSWORD>` with the actual leaked password when you build
`/tmp/replacements.txt` locally — it is deliberately not written into this
repo.

## 2. Rotate the exposed credential

The Jetson SSH **and** VNC password was public in this repo's history. Rewriting
history does not un-leak it from any clone that already exists — **rotate it**
and treat it as compromised. Switch SSH to key-based auth (`ssh-copy-id`).

## 3. After the rewrite

- Everyone with a clone must re-clone; old history is invalid.
- The `192.168.55.x` addresses in the tree are NVIDIA Jetson USB-gadget
  defaults (identical on every deployment) and are intentionally kept.
- Set the real Jetson LAN IP in `cyclonedds.xml` (`JETSON_LAN_IP`
  placeholder) for your own deployment; do not commit a site IP.
