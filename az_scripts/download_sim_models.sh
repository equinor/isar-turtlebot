#!/bin/bash

SCRIPTDIR="$( cd "$( dirname "${BASH_SOURCE}" )" >/dev/null && pwd )"

ACCOUNT_NAME="turtlebotsimmodels"

# Download models
az storage file download-batch \
    --destination "$SCRIPTDIR/../models/" \
    --source "data/models" \
    --account-key $STORAGE_ACCOUNT_KEY \
    --account-name $ACCOUNT_NAME

# Download maps
az storage file download-batch \
    --destination "$SCRIPTDIR/../maps/" \
    --source "data/maps" \
    --account-key $STORAGE_ACCOUNT_KEY \
    --account-name $ACCOUNT_NAME


# Download world files
az storage file download-batch \
    --destination "$SCRIPTDIR/../worlds/" \
    --source "data/worlds" \
    --account-key $STORAGE_ACCOUNT_KEY \
    --account-name $ACCOUNT_NAME


# Download config files
az storage file download-batch \
    --destination "$SCRIPTDIR/../config/" \
    --source "data/config" \
    --account-key $STORAGE_ACCOUNT_KEY \
    --account-name $ACCOUNT_NAME
