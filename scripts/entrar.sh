#!/usr/bin/env bash
# Entra no container do lar_gazebo que já está rodando, sem precisar
# descobrir o ID manualmente com docker ps.

CONTAINER_ID=$(docker ps --filter "ancestor=lar-gazebo:noetic" --format "{{.ID}}" | head -n 1)

if [ -z "$CONTAINER_ID" ]; then
  echo "Nenhum container lar_gazebo está rodando."
  echo "Rode primeiro: ./scripts/run_husky.sh"
  exit 1
fi

docker exec -it "$CONTAINER_ID" bash
