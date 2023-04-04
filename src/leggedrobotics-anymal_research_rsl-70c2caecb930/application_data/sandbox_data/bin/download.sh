#!/usr/bin/env bash

URL="https://data.anybotics.com"
WORKING_DIR=$1
DATA_PACKAGE=$2
USER=$3
PASSWORD=$4

# Check if the website is available.
RESPONSE="$(curl -Is ${URL} | head -1)"
if [[ -z "${RESPONSE}" ]]; then
  echo ${URL}" not reachable."
  exit;
fi

# Download available data if newer.
wget \
  --user ${USER} \
  --password ${PASSWORD} \
  --directory-prefix ${WORKING_DIR} \
  --no-host-directories \
  --recursive \
  --no-parent \
  --timestamping \
  --reject "index.html*" \
  ${URL}/${DATA_PACKAGE}/ \
  > ${WORKING_DIR}/download.log 2>&1

# Loop over all archives in the download folder and extract them into a dedicated folder.
for ARCHIVE_DIR in ${WORKING_DIR}/${DATA_PACKAGE}/*.tar.gz; do
  ARCHIVE_NAME=`expr ${ARCHIVE_DIR} : '\(.*\).tar.gz'`
  mkdir ${ARCHIVE_NAME} 2>/dev/null
  tar -xzf ${ARCHIVE_DIR} -C ${ARCHIVE_NAME}
done

# Loop over all folders in the download folder and move them to the working directory.
for DOWNLOAD_FOLDER_DIR in ${WORKING_DIR}/${DATA_PACKAGE}/*/; do
  FOLDER_NAME="$(basename ${DOWNLOAD_FOLDER_DIR})"
  TARGET_FOLDER_DIR="${WORKING_DIR}/${FOLDER_NAME}"
  echo ${TARGET_FOLDER_DIR}
  if [ -d ${TARGET_FOLDER_DIR} ]; then
    rm -rf ${TARGET_FOLDER_DIR}
  fi
  mv ${DOWNLOAD_FOLDER_DIR} ${WORKING_DIR}
done
