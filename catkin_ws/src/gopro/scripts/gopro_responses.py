status_matrix = {
    'camera/se': {
        'preview': {
            'first': 37,
            'second': 38,
            'translate': {
                '0': 'off',
                '1': 'on'
            }
        },
        'batt1': {
            'first': 38,
            'second': 40,
            'translate': 'hex_to_dec'
        }
    },
    'camera/sx': {  # the first 62 bytes of sx are almost the same as se
        'mode': {
            'first': 2,
            'second': 4,
            'translate': {
                '00': 'video',
                '01': 'still',
                '02': 'burst',
                '03': 'timelapse',
                '07': 'settings'
            }
        },
        'default_mode': {
            'first': 6,
            'second': 8,
            'translate': {
                '00': 'video',
                '01': 'still',
                '02': 'burst',
                '03': 'timelapse'
            }
        },
        'spot_meter': {
            'first': 8,
            'second': 10,
            'translate': {
                '00': 'off',
                '01': 'on'
            }
        },
        'timelapse_interval': {
            'first': 10,
            'second': 12,
            'translate': 'hex_to_dec'
        },
        'auto_off': {
            'first': 12,
            'second': 14,
            'translate': {
                '00': 'never',
                '01': '1 minute',
                '02': '2 minutes',
                '03': '5 minutes'
            }
        },
        'fov': {
            'first': 14,
            'second': 16,
            'translate': {
                '00': '170',
                '01': '127',
                '02': '90'
            }
        },
        'picres': {
            'first': 17,
            'second': 18,
            'translate': {
                '3': '5MP med',
                '6': '7MP med',
                '4': '7MP wide',
                '5': '12MP wide'
            }
        },
        'mins_elapsed': {
            'first': 26,
            'second': 28,
            'translate': 'hex_to_dec'
        },
        'secs_elapsed': {
            'first': 28,
            'second': 30,
            'translate': 'hex_to_dec'
        },
        'orientation': {
            'first': 37,
            'second': 38,
            'translate': {
                # looks like this really should just be the third bit
                '0': 'up',
                '1': '',
                '4': 'down'
            }
        },
        'charging': {
            'first': 39,
            'second': 40,
            'translate': {
                '2': 'no',  # 2 only shows up temporarily
                '3': 'no',
                '4': 'yes'
            }
        },
        'pics_remaining': {
            'first': 42,
            'second': 46,
            'translate': 'hex_to_dec'
        },
        'n_pics': {
            'first': 46,
            'second': 50,
            'translate': 'hex_to_dec'
        },
        'mins_remaining': {
            'first': 50,
            'second': 54,
            'translate': 'hex_to_dec'
        },
        'n_vids': {
            'first': 54,
            'second': 58,
            'translate': 'hex_to_dec'
        },
        'record': {
            'first': 58,
            'second': 60,
            'translate': {
                '00': 'off',
                '01': 'on'
            }
        },
        'low_light': {
            'first': 60,
            'second': 61,
            'translate': {
                '0': 'off',
                '4': 'on'
            }
        },
        'protune': {
            'first': 61,
            'second': 62,
            'translate': {
                '0': 'off',  # seems to be only while in the menu
                '2': 'on',  # seems to be only while in the menu
                '4': 'off',
                '6': 'on'
            }
        },
        'white_balance': {
            'first': 69,
            'second': 70,
            'translate': {
                '0': 'auto',
                '1': '3000K',
                '2': '5500K',
                '3': '6500K',
                '4': 'raw'
            }
        },
        'looping': {
            'first': 74,
            'second': 76,
            'translate': {
                '00': 'off',
                '01': '5 minutes',
                '02': '20 minutes',
                '03': '60 minutes',
                '04': '120 minutes',
                '05': 'max'
            }
        },
        'batt2': {
            'first': 90,
            'second': 92,
            'translate': 'hex_to_dec'
        },
        'overheated': {  # experimental
            'first': 92,
            'second': 93,
            'translate': {
                '0': 'false',
                '4': 'true'
            }
        },
        'attachment': {
            'first': 93,
            'second': 94,
            'translate': {
                '0': 'none',
                '4': 'LCD',
                '8': 'battery'
            }
        },
        'vidres': {
            'first': 100,
            'second': 102,
            'translate': {
                '00': 'WVGA',
                '01': '720p',
                '02': '960p',
                '03': '1080p',
                '04': '1440p',
                '05': '2.7K',
                '06': '2.7K 17:9 Cinema',
                '07': '4K',
                '08': '4K 17:9 Cinema',
                '09': '1080p SuperView',
                '0a': '720p SuperView'
            }
        },
        'fps': {
            'first': 102,
            'second': 104,
            'translate': {
                '00': '12',
                '01': '15',
                '02': '24',
                '03': '25',
                '04': '30',
                '05': '48',
                '06': '50',
                '07': '60',
                '08': '100',
                '09': '120',
                '0a': '240'
            }
        }
    }
}

command_matrix = {
    'power': {
        'cmd': 'bacpac/PW',
        'translate': {
            'sleep': '00',
            'on': '01'
        }
    },
    'record': {
        'cmd': 'camera/SH',
        'translate': {
            'off': '00',
            'on': '01'
        }
    },
    'preview': {
        'cmd': 'camera/PV',
        'translate': {
            'off': '00',
            'on': '02'
        }
    },
    'orientation': {
        'cmd': 'camera/UP',
        'translate': {
            'up': '00',
            'down': '01'
        }
    },
    'mode': {
        'cmd': 'camera/CM',
        'translate': {
            'video': '00',
            'still': '01',
            'burst': '02',
            'timelapse': '03',
            'timer': '04',
            'hdmiout': '05'
        }
    },
    'volume': {
        'cmd': 'camera/BS',
        'translate': {
            '0': '00',
            '70': '01',
            '100': '02'
        }
    },
    'locate': {
        'cmd': 'camera/LL',
        'translate': {
            'off': '00',
            'on': '01'
        }
    },
    'picres': {
        'cmd': 'camera/PR',
        'translate': {
            '11MP wide': '00',
            '8MP med': '01',
            '5MP wide': '02',
            '5MP med': '03',
            '7MP wide': '04',
            '12MP wide': '05',
            '7MP med': '06'
        }
    },
    'vidres': {
        'cmd': 'camera/VV',
        'translate': {
            'WVGA': '00',
            '720p': '01',
            '960p': '02',
            '1080p': '03',
            '1440p': '04',
            '2.7K': '05',
            '2.7K 17:9 Cinema': '06',
            '4K': '07',
            '4K 17:9 Cinema': '08',
            '1080p SuperView': '09',
            '720p SuperView': '0a'
        }
    },
    'fov': {
        'cmd': 'camera/FV',
        'translate': {
            '170': '00',
            '127': '01',
            '90': '02'
        }
    },
    'fps': {
        'cmd': 'camera/FS',
        'translate': {
            '12': '00',
            '12.5': '0b',
            '15': '01',
            '24': '02',
            '25': '03',
            '30': '04',
            '48': '05',
            '50': '06',
            '60': '07',
            '100': '08',
            '120': '09',
            '240': '0a'
        }
    },
    'looping': {
        'cmd': 'camera/LO',
        'translate': {
            'off': '00',
            '5 minutes': '01',
            '20 minutes': '02',
            '60 minutes': '03',
            '120 minutes': '04',
            'max': '05'
        }
    },
    'protune': {
        'cmd': 'camera/PT',
        'translate': {
            'off': '00',
            'on': '01'
        }
    },
    'delete_last': {
        'cmd': 'camera/DL'
    },
    'delete_all': {
        'cmd': 'camera/DA'
    }
}
