package goroslib

import (
    "testing"
    "fmt"

    "github.com/stretchr/testify/require"
)

func TestActionClient(t *testing.T) {
	for _, provider := range []string{
		"cpp",
		//"go",
	} {
		t.Run(provider, func(t *testing.T) {
			m, err := newContainerMaster()
			require.NoError(t, err)
			defer m.close()

            switch provider {
			case "cpp":
				p, err := newContainer("node-actionserver", m.IP())
				require.NoError(t, err)
                defer p.close()
            }

            n, err := NewNode(NodeConf{
				Namespace:     "/myns",
				Name:          "goroslib",
				MasterAddress: m.IP() + ":11311",
			})
			require.NoError(t, err)
			defer n.Close()

            topics, _ := n.GetTopics()
            fmt.Println(topics)

            serv, _ := n.GetServices()
            fmt.Println(serv)

        })
    }
}
