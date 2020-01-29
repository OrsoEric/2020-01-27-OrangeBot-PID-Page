{
    "targets": [{
        "target_name": "OrangebotNodeCpp",
        "cflags!": [ "-fno-exceptions" ],
        "cflags_cc!": [ "-fno-exceptions" ],
        "sources": [
            "src/orangebot_node_bindings.cpp",
			"src/panopticon.cpp",
			"src/ob.cpp",
			"src/uniparser.cpp",
			"src/debug.cpp"
        ],
        'include_dirs': [
            "<!@(node -p \"require('node-addon-api').include\")"
        ],
        'libraries': [],
        'dependencies': [
            "<!(node -p \"require('node-addon-api').gyp\")"
        ],
        'defines': [ 'NAPI_DISABLE_CPP_EXCEPTIONS' ]
    }]
}
