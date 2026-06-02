#!/usr/bin/env python3
# -*- coding: utf-8 -*-

##########################################
# WARNING: DO NOT MODIFY - THIS HAS NO RELATIONSHIP WITH CODE
##########################################

import os
import re
import json
import sys
import datetime
import urllib.request
from typing import List, Optional
from datetime import datetime, timezone, timedelta

def env(name: str, default: str = "") -> str:
    return os.environ.get(name, default)

def strip_project_dir_prefix(line: str, project_dir: str) -> str:
    # Remove "<CI_PROJECT_DIR>/" prefix if present
    prefix = project_dir.rstrip("/") + "/"
    if line.startswith(prefix):
        return line[len(prefix):]
    return line

def load_lines(log_path: str) -> List[str]:
    with open(log_path, "r", errors="ignore") as f:
        return [ln.rstrip("\n") for ln in f]

def extract_memory_table(lines: List[str]) -> str:
    """
    Find the last 'Memory region' block and return a few lines after it.
    """
    idx = -1
    for i in range(len(lines) - 1, -1, -1):
        if lines[i].startswith("Memory region"):
            idx = i
            break
    if idx < 0:
        return ""

    # Grab up to next 12 lines (usually enough for the table)
    block = lines[idx: idx + 12]
    # Trim trailing empty lines
    while block and not block[-1].strip():
        block.pop()
    return "\n".join(block).strip()

def extract_diagnostics(lines: List[str], mode: str) -> str:
    """
    mode:
      - success: include info/warning/note (and any diagnostics)
      - failed:  include fatal/error/warning/note/info + linker errors, etc.
    """
    if mode == "success":
        pattern = re.compile(r"(info:|warning:|note:)", re.IGNORECASE)
    else:
        pattern = re.compile(
            r"(fatal error:|error:|warning:|note:|info:|undefined reference|collect2: error|ld: error|"
            r"CMake Error|ninja: build stopped|FAILED:)",
            re.IGNORECASE
        )

    matched = [ln for ln in lines if pattern.search(ln)]

    # Keep it reasonably sized
    if matched:
        text = "\n".join(matched[-120:])
    else:
        # Fallback: show tail if nothing matched
        text = "\n".join(lines[-120:]).strip() or ("Build succeeded" if mode == "success" else "Build failed")

    # Truncate to avoid Feishu limits
    max_chars = 3800
    if len(text) > max_chars:
        text = "…(truncated)\n" + text[-max_chars:]
    return text

def read_text_if_exists(path: str) -> str:
    try:
        with open(path, "r", errors="ignore") as f:
            return f.read().strip()
    except FileNotFoundError:
        return ""

def post_json(url: str, payload: dict) -> None:
    req = urllib.request.Request(
        url,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        print(resp.read().decode("utf-8", errors="ignore"))

def main() -> None:
    status = (sys.argv[1] if len(sys.argv) > 1 else "failed").lower()
    if status not in ("success", "failed"):
        status = "failed"

    webhook = env("FEISHU_WEBHOOK")
    if not webhook:
        raise SystemExit("FEISHU_WEBHOOK is not set")

    build_dir = env("BUILD_DIR", "build")
    project_dir = env("CI_PROJECT_DIR", "")

    log_path = os.path.join(build_dir, "build.log")
    try:
        raw_lines = load_lines(log_path)
    except FileNotFoundError:
        raw_lines = ["build.log not found"]

    # Strip CI_PROJECT_DIR prefix from every line (both success and failure)
    lines = [strip_project_dir_prefix(ln, project_dir) for ln in raw_lines]

    diag = extract_diagnostics(lines, status)
    mem = extract_memory_table(lines)

    if mem:
        message = diag + "\n\n" + mem
    else:
        message = diag

    # -------- metadata --------
    build_number  = env("CI_PIPELINE_ID")
    repo_name     = env("CI_PROJECT_PATH")
    repo_url      = env("CI_PROJECT_URL")
    branch_name   = env("CI_COMMIT_REF_NAME")
    pipeline_url  = env("CI_PIPELINE_URL")
    commit_id     = env("CI_COMMIT_SHORT_SHA")
    commit_author = env("GITLAB_USER_NAME") or env("CI_COMMIT_AUTHOR")
    commit_message = env("CI_COMMIT_MESSAGE").strip()
    if len(commit_message) > 800:
        commit_message = commit_message[:800] + "\n…(truncated)"
    CN_TZ = timezone(timedelta(hours=8), name="UTC+8")
    event_time = datetime.now(CN_TZ).strftime("%Y-%m-%d %H:%M:%S (UTC+8)")

    # Build job URL for artifacts (NOT current notify job)
    build_job_url = read_text_if_exists(os.path.join(build_dir, "ci_build_job_url.txt"))
    artifacts_browse_url = (build_job_url + "/artifacts/browse") if build_job_url else ""

    # -------- style switch --------
    if status == "success":
        header_title = "OmniX Git CI/CD Compile SUCCESS"
        tag_text = "OK"
        tag_color = "green"
        template = "green"
        banner_left_bg = "green-50"
        banner_left_text = "**<font color='green'>✅ PIPELINE SUCCESS</font>**"
        banner_right_bg = "blue-50"
        banner_right_title = "**<font color='blue'>🕒 Success Time</font>**"
        icon_token = "check-circle_outlined"
    else:
        header_title = "OmniX Git CI/CD Compile FAILED"
        tag_text = "ERROR"
        tag_color = "red"
        template = "red"
        banner_left_bg = "red-50"
        banner_left_text = "**<font color='red'>❌ PIPELINE FAILED</font>**"
        banner_right_bg = "orange-50"
        banner_right_title = "**<font color='orange'>🕒 Failed Time</font>**"
        icon_token = "alert-circle_outlined"

    card = {
        "schema": "2.0",
        "config": {
            "update_multi": True,
            "style": {
                "text_size": {
                    "normal_v2": {"default": "normal", "pc": "normal", "mobile": "heading"}
                }
            }
        },
        "body": {
            "direction": "vertical",
            "elements": [
                {
                    "tag": "column_set",
                    "flex_mode": "stretch",
                    "horizontal_spacing": "12px",
                    "horizontal_align": "left",
                    "columns": [
                        {
                            "tag": "column",
                            "width": "weighted",
                            "background_style": banner_left_bg,
                            "elements": [
                                {"tag": "markdown", "content": banner_left_text, "text_align": "center", "text_size": "normal_v2"},
                                {"tag": "markdown", "content": f"<font color='grey'>Build #{build_number}</font>", "text_align": "center", "text_size": "normal"}
                            ],
                            "padding": "12px 12px 12px 12px",
                            "vertical_spacing": "4px",
                            "weight": 1
                        },
                        {
                            "tag": "column",
                            "width": "weighted",
                            "background_style": banner_right_bg,
                            "elements": [
                                {"tag": "markdown", "content": banner_right_title, "text_align": "center", "text_size": "normal_v2"},
                                {"tag": "markdown", "content": f"<font color='grey'>{event_time}</font>", "text_align": "center", "text_size": "normal"}
                            ],
                            "padding": "12px 12px 12px 12px",
                            "vertical_spacing": "4px",
                            "weight": 1
                        }
                    ]
                },
                {
                    "tag": "column_set",
                    "flex_mode": "stretch",
                    "horizontal_spacing": "12px",
                    "horizontal_align": "left",
                    "columns": [
                        {"tag": "column", "width": "weighted", "elements": [{"tag": "markdown", "content": f"**REPO**\n{repo_name}", "text_align": "left", "text_size": "normal_v2"}], "weight": 1},
                        {"tag": "column", "width": "weighted", "elements": [{"tag": "markdown", "content": f"**Branch**\n{branch_name}", "text_align": "left", "text_size": "normal_v2"}], "weight": 1}
                    ]
                },
                {
                    "tag": "column_set",
                    "flex_mode": "stretch",
                    "horizontal_spacing": "12px",
                    "horizontal_align": "left",
                    "columns": [
                        {"tag": "column", "width": "weighted", "elements": [{"tag": "markdown", "content": f"**Commit by**\n{commit_author}", "text_align": "left", "text_size": "normal_v2"}], "weight": 1},
                        {"tag": "column", "width": "weighted", "elements": [{"tag": "markdown", "content": f"**Commit ID**\n{commit_id}"}], "weight": 1}
                    ]
                },
                {
                "tag": "markdown",
                "content": f"**Commit Message**\n```text\n{commit_message}\n```",
                "text_align": "left",
                "text_size": "normal_v2"
                },
                {
                "tag": "markdown",
                "content": f"**Compile Message**\n```text\n{message}\n```",
                "text_align": "left",
                "text_size": "normal_v2"
                },
                {
                    "tag": "column_set",
                    "flex_mode": "stretch",
                    "horizontal_spacing": "8px",
                    "horizontal_align": "left",
                    "columns": [
                        {
                            "tag": "column",
                            "width": "auto",
                            "elements": [{
                                "tag": "button",
                                "text": {"tag": "plain_text", "content": "查看仓库"},
                                "type": "primary_filled",
                                "width": "fill",
                                "behaviors": [{"type": "open_url", "default_url": repo_url, "pc_url": "", "ios_url": "", "android_url": ""}]
                            }]
                        },
                        {
                            "tag": "column",
                            "width": "auto",
                            "elements": [{
                                "tag": "button",
                                "text": {"tag": "plain_text", "content": "查看Pipeline"},
                                "type": "default",
                                "width": "fill",
                                "behaviors": [{"type": "open_url", "default_url": pipeline_url, "pc_url": "", "ios_url": "", "android_url": ""}]
                            }]
                        },
                        {
                            "tag": "column",
                            "width": "auto",
                            "elements": [{
                                "tag": "button",
                                "text": {"tag": "plain_text", "content": "查看Artifacts"},
                                "type": "default",
                                "width": "fill",
                                "behaviors": [{"type": "open_url", "default_url": artifacts_browse_url, "pc_url": "", "ios_url": "", "android_url": ""}]
                            }]
                        }
                    ]
                }
            ]
        },
        "header": {
            "title": {"tag": "plain_text", "content": header_title},
            "subtitle": {"tag": "plain_text", "content": ""},
            "text_tag_list": [{"tag": "text_tag", "text": {"tag": "plain_text", "content": tag_text}, "color": tag_color}],
            "template": template,
            "icon": {"tag": "standard_icon", "token": icon_token},
            "padding": "12px 8px 12px 8px"
        }
    }

    payload = {"msg_type": "interactive", "card": card}
    post_json(webhook, payload)

if __name__ == "__main__":
    main()